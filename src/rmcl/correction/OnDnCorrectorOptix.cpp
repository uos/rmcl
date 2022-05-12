#include "rmcl/correction/OnDnCorrectorOptix.hpp"

#include "rmcl/correction/optix/OnDnCorrectProgramRW.hpp"
#include "rmcl/correction/optix/OnDnCorrectProgramSW.hpp"

#include "rmcl/correction/optix/CorrectionDataOptix.hpp"

#include <rmagine/math/math_batched.cuh>

#include <rmcl/math/math_batched.cuh>


// DEBUG
#include <rmagine/util/prints.h>


namespace rm = rmagine;

namespace rmcl 
{

OnDnCorrectorOptix::OnDnCorrectorOptix(
    rmagine::OptixMapPtr map)
:Base(map)
{
    programs.resize(2);
    programs[0].reset(new OnDnCorrectProgramRW(map));
    programs[1].reset(new OnDnCorrectProgramSW(map));

    // CUDA_CHECK( cudaStreamCreate(&m_stream) );
    m_svd.reset(new rm::SVDCuda(Base::m_stream));

    // default params
    CorrectionParams params;
    setParams(params);
}

void OnDnCorrectorOptix::setParams(
    const CorrectionParams& params)
{
    rm::Memory<CorrectionParams, rm::RAM> params_ram(1);
    params_ram[0] = params;
    m_params = params_ram;
}

void OnDnCorrectorOptix::setInputData(
    const rmagine::MemoryView<float, rmagine::RAM>& ranges
)
{
    m_ranges = ranges;
}

void OnDnCorrectorOptix::setInputData(
    const rmagine::MemoryView<float, rmagine::VRAM_CUDA>& ranges)
{
    m_ranges = ranges;
}

CorrectionResults<rm::VRAM_CUDA> OnDnCorrectorOptix::correct(
    const rm::MemoryView<rm::Transform, rm::VRAM_CUDA>& Tbms) const
{
    // std::cout << "Start correction." << std::endl;
    CorrectionResults<rm::VRAM_CUDA> res;
    res.Ncorr.resize(Tbms.size());
    res.Tdelta.resize(Tbms.size());

    if(Tbms.size() == 0)
    {
        return res;
    }

    rm::Memory<rm::Vector, rm::VRAM_CUDA> m1(Tbms.size());
    rm::Memory<rm::Vector, rm::VRAM_CUDA> m2(Tbms.size());
    rm::Memory<rm::Matrix3x3, rm::VRAM_CUDA> Cs(Tbms.size());
    rm::Memory<rm::Matrix3x3, rm::VRAM_CUDA> Us(Cs.size());
    rm::Memory<rm::Matrix3x3, rm::VRAM_CUDA> Vs(Cs.size());

    // TODO how to make this dynamic somehow
    constexpr unsigned int POSE_SWITCH = 1024 * 8;
    // constexpr unsigned int POSE_SWITCH = 0;

    if(Tbms.size() > POSE_SWITCH)
    {
        // scanwise parallelization
        computeMeansCovsSW(Tbms, m1, m2, Cs, res.Ncorr);
    } else {
        // raywise parallelization
        computeMeansCovsRW(Tbms, m1, m2, Cs, res.Ncorr);
    }

    // Singular value decomposition
    
    m_svd->calcUV(Cs, Us, Vs);
    auto Rs = rm::multNxN(Us, rm::transpose(Vs));
    auto ts = rm::subNxN(m1, rm::multNxN(Rs, m2));

    rm::pack(Rs, ts, res.Tdelta);

    return res;
}

/// PRIVATE
void OnDnCorrectorOptix::computeMeansCovsRW(
    const rm::MemoryView<rm::Transform, rm::VRAM_CUDA>& Tbm,
    rm::MemoryView<rm::Vector, rm::VRAM_CUDA>& m1, // from, dataset
    rm::MemoryView<rm::Vector, rm::VRAM_CUDA>& m2, // to, model
    rm::MemoryView<rm::Matrix3x3, rm::VRAM_CUDA>& Cs,
    rm::MemoryView<unsigned int, rm::VRAM_CUDA>& Ncorr
    ) const
{
    rm::Memory<rm::OnDnModel_<rm::VRAM_CUDA>, rm::VRAM_CUDA> model(1);
    copy(m_model, model, m_stream);

    size_t scanSize = m_width * m_height;
    // size_t scanSize = m_model_ram->width * m_model_ram->height;
    size_t Nrays = Tbm.size() * scanSize;

    rm::Memory<unsigned int,    rm::VRAM_CUDA> corr_valid(Nrays);
    rm::Memory<rm::Vector, rm::VRAM_CUDA> model_points(Nrays);
    rm::Memory<rm::Vector, rm::VRAM_CUDA> dataset_points(Nrays);

    rm::Memory<OnDnCorrectionDataRW, rm::RAM> mem(1);
    mem->model = model.raw();
    mem->ranges = m_ranges.raw();
    mem->Tsb = m_Tsb.raw();
    mem->Tbm = Tbm.raw();
    mem->Nposes = Tbm.size();
    mem->params = m_params.raw();
    mem->handle = m_map->as.handle;
    mem->corr_valid = corr_valid.raw();
    mem->model_points = model_points.raw();
    mem->dataset_points = dataset_points.raw();

    rm::Memory<OnDnCorrectionDataRW, rm::VRAM_CUDA> d_mem(1);
    copy(mem, d_mem, Base::m_stream);

    rm::OptixProgramPtr program = programs[0];

    OPTIX_CHECK( optixLaunch(
        program->pipeline, 
        Base::m_stream, 
        reinterpret_cast<CUdeviceptr>(d_mem.raw()), 
        sizeof( OnDnCorrectionDataRW ), 
        &program->sbt,
        m_width, // width Xdim
        m_height, // height Ydim
        Tbm.size()// depth Zdim
        ));

    cudaStreamSynchronize(m_stream);

    rm::sumBatched(corr_valid, Ncorr);
    meanBatched(dataset_points, corr_valid, Ncorr, m1);
    meanBatched(model_points, corr_valid, Ncorr, m2);

    covFancyBatched(model_points, m2, 
            dataset_points, m1, 
            corr_valid, Ncorr, Cs);
}

void OnDnCorrectorOptix::computeMeansCovsSW(
    const rm::MemoryView<rm::Transform, rm::VRAM_CUDA>& Tbm,
    rm::MemoryView<rm::Vector, rm::VRAM_CUDA>& m1,
    rm::MemoryView<rm::Vector, rm::VRAM_CUDA>& m2,
    rm::MemoryView<rm::Matrix3x3, rm::VRAM_CUDA>& Cs,
    rm::MemoryView<unsigned int, rm::VRAM_CUDA>& Ncorr
    ) const
{
    rm::Memory<rm::OnDnModel_<rm::VRAM_CUDA>, rm::VRAM_CUDA> model(1);
    copy(m_model, model, m_stream);

    // Create Optix Memory structure
    rm::Memory<OnDnCorrectionDataSW, rm::RAM_CUDA> mem(1);
    mem->model = model.raw();
    mem->ranges = m_ranges.raw();
    mem->Tbm = Tbm.raw();
    mem->Tsb = m_Tsb.raw();
    mem->Nposes = Tbm.size();
    mem->params = m_params.raw();
    mem->handle = m_map->as.handle;
    mem->C = Cs.raw();
    mem->m1 = m1.raw(); // Sim
    mem->m2 = m2.raw(); // Real
    mem->Ncorr = Ncorr.raw();

    rm::Memory<OnDnCorrectionDataSW, rm::VRAM_CUDA> d_mem(1);
    copy(mem, d_mem, m_stream);

    rm::OptixProgramPtr program = programs[1];

    OPTIX_CHECK( optixLaunch(
        program->pipeline, 
        m_stream, 
        reinterpret_cast<CUdeviceptr>(d_mem.raw()), 
        sizeof( OnDnCorrectionDataSW ), 
        &program->sbt, 
        Tbm.size(),
        1,
        1
        ));

    cudaStreamSynchronize(m_stream);
}

} // namespace rmcl