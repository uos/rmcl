#include "rmcl/correction/PinholeCorrectorOptix.hpp"

// #include "rmcl/correction/optix/PinholeCorrectProgramRW.hpp"
// #include "rmcl/correction/optix/PinholeCorrectProgramSW.hpp"

#include "rmcl/correction/optix/corr_pipelines.h"

#include "rmcl/correction/optix/CorrectionDataOptix.hpp"

#include <rmagine/math/math_batched.cuh>

#include <rmcl/math/math_batched.cuh>
#include <rmcl/math/math.cuh>


// DEBUG
#include <rmagine/util/prints.h>
#include <rmagine/util/StopWatch.hpp>

#include <optix_stubs.h>


namespace rm = rmagine;

namespace rmcl 
{

PinholeCorrectorOptix::PinholeCorrectorOptix(
    rmagine::OptixMapPtr map)
:Base(map)
{
    m_svd = std::make_shared<rm::SVDCuda>(Base::m_stream);

    // default params
    CorrectionParams params;
    setParams(params);
}

void PinholeCorrectorOptix::setParams(
    const CorrectionParams& params)
{
    rm::Memory<CorrectionParams, rm::RAM> params_ram(1);
    params_ram[0] = params;
    m_params = params_ram;
}

void PinholeCorrectorOptix::setInputData(
    const rmagine::MemoryView<float, rmagine::VRAM_CUDA>& ranges)
{
    m_ranges = ranges;
}

void PinholeCorrectorOptix::setOptical(bool optical)
{   
    m_optical = optical;
}

CorrectionResults<rm::VRAM_CUDA> PinholeCorrectorOptix::correct(
    const rm::MemoryView<rm::Transform, rm::VRAM_CUDA>& Tbms) const
{
    auto optix_ctx = m_map->context();
    auto cuda_ctx = optix_ctx->getCudaContext();
    if(!cuda_ctx->isActive())
    {
        std::cout << "[SphereCorrectorOptix::correct() Need to activate map context" << std::endl;
        cuda_ctx->use();
    }

    CorrectionResults<rm::VRAM_CUDA> res;
    if(Tbms.size() == 0)
    {
        return res;
    }

    res.Ncorr.resize(Tbms.size());
    res.Tdelta.resize(Tbms.size());

    rm::Memory<rm::Vector, rm::VRAM_CUDA> ms(Tbms.size());
    rm::Memory<rm::Vector, rm::VRAM_CUDA> ds(Tbms.size());
    rm::Memory<rm::Matrix3x3, rm::VRAM_CUDA> Cs(Tbms.size());
    rm::Memory<rm::Matrix3x3, rm::VRAM_CUDA> Us(Cs.size());
    rm::Memory<rm::Matrix3x3, rm::VRAM_CUDA> Vs(Cs.size());
    
    compute_covs(Tbms, ms, ds, Cs, res.Ncorr);

    static CorrectionCuda corr(m_svd);
    corr.correction_from_covs(ms, ds, Cs, res.Ncorr, res.Tdelta);

    return res;
}

static size_t corr_additional_bytes_per_pose()
{
    size_t required_bytes_per_pose = 0;
    // m1
    required_bytes_per_pose += sizeof(rm::Vector);
    // m2
    required_bytes_per_pose += sizeof(rm::Vector);
    // Cs
    required_bytes_per_pose += sizeof(rm::Matrix3x3);
    // Us
    required_bytes_per_pose += sizeof(rm::Matrix3x3);
    // Vs
    required_bytes_per_pose += sizeof(rm::Matrix3x3);

    return required_bytes_per_pose;
}

static size_t rw_mode_additional_bytes_per_ray()
{
    size_t required_bytes_per_ray_rw = 0;
    // corr_valid
    required_bytes_per_ray_rw += sizeof(unsigned int);
    // model points
    required_bytes_per_ray_rw += sizeof(rm::Vector);
    // dataset points
    required_bytes_per_ray_rw += sizeof(rm::Vector);

    return required_bytes_per_ray_rw;
}

static size_t rw_mode_additional_bytes_per_pose(size_t Nrays)
{
    return Nrays * rw_mode_additional_bytes_per_ray();
}

static size_t rw_mode_additional_bytes(size_t Nposes, size_t Nrays)
{
    return Nposes * rw_mode_additional_bytes_per_pose(Nrays);
}

void PinholeCorrectorOptix::compute_covs(
    const rmagine::MemoryView<rmagine::Transform, rmagine::VRAM_CUDA>& Tbms,
    rmagine::MemoryView<rmagine::Vector, rmagine::VRAM_CUDA>& ms,
    rmagine::MemoryView<rmagine::Vector, rmagine::VRAM_CUDA>& ds,
    rmagine::MemoryView<rmagine::Matrix3x3, rmagine::VRAM_CUDA>& Cs,
    rmagine::MemoryView<unsigned int, rmagine::VRAM_CUDA>& Ncorr) const
{
    size_t scanSize = m_width * m_height;
    size_t Nrays = Tbms.size() * scanSize;


    size_t general_bytes = corr_additional_bytes_per_pose();
    size_t rw_mode_bytes = rw_mode_additional_bytes_per_pose(scanSize);

    size_t free_bytes, total_bytes;
    cudaMemGetInfo(&free_bytes, &total_bytes);

    size_t max_poses_rw = free_bytes / (general_bytes + rw_mode_bytes);

    // fix estimation with 3/4
    max_poses_rw *= 3;
    max_poses_rw /= 4;


    // TODO how to make this dynamic somehow
    constexpr unsigned int POSE_SWITCH = 1024 * 8;

    if(Tbms.size() > POSE_SWITCH)
    {
        // std::cout << "Scanwise" << std::endl;
        // scanwise parallelization
        computeMeansCovsSW(Tbms, ds, ms, Cs, Ncorr);
    } else {
        if(Tbms.size() < max_poses_rw)
        {
            // std::cout << "Raywise" << std::endl;
            computeMeansCovsRW(Tbms, ds, ms, Cs, Ncorr);
        } else {
            // std::cout << "Raywise splitted" << std::endl;
            // split
            // std::cout << "Need to split!" << std::endl;
            
            const size_t chunkSize = 512;

            for(size_t i=0; i<Tbms.size(); i += chunkSize)
            {
                size_t chunk_start = i;
                size_t chunk_end = i + chunkSize;
                if(chunk_end > Tbms.size())
                {
                    chunk_end = Tbms.size();
                }

                // slice
                auto Tbms_ = Tbms(chunk_start, chunk_end);
                auto ms_ = ms(chunk_start, chunk_end);
                auto ds_ = ds(chunk_start, chunk_end);
                auto Cs_ = Cs(chunk_start, chunk_end);
                auto Ncorr_ = Ncorr(chunk_start, chunk_end);

                computeMeansCovsRW(Tbms_, ds_, ms_, Cs_, Ncorr_);
            }
        }
    }

    if(Tbms.size() > POSE_SWITCH)
    {
        // scanwise parallelization
        computeMeansCovsSW(Tbms, ds, ms, Cs, Ncorr);
    } else {
        // raywise parallelization
        computeMeansCovsRW(Tbms, ds, ms, Cs, Ncorr);
    }
}

void PinholeCorrectorOptix::compute_covs(
    const rmagine::MemoryView<rmagine::Transform, rmagine::VRAM_CUDA>& Tbms,
    CorrectionPreResults<rmagine::VRAM_CUDA>& res
) const
{
    compute_covs(Tbms, res.ms, res.ds, res.Cs, res.Ncorr);
}

CorrectionPreResults<rmagine::VRAM_CUDA> PinholeCorrectorOptix::compute_covs(
    const rmagine::MemoryView<rmagine::Transform, rmagine::VRAM_CUDA>& Tbms
) const
{
    CorrectionPreResults<rmagine::VRAM_CUDA> res;
    res.ms.resize(Tbms.size());
    res.ds.resize(Tbms.size());
    res.Cs.resize(Tbms.size());
    res.Ncorr.resize(Tbms.size());

    compute_covs(Tbms, res);

    return res;
}

/// PRIVATE
void PinholeCorrectorOptix::computeMeansCovsRW(
    const rm::MemoryView<rm::Transform, rm::VRAM_CUDA>& Tbm,
    rm::MemoryView<rm::Vector, rm::VRAM_CUDA>& m1, // from, dataset
    rm::MemoryView<rm::Vector, rm::VRAM_CUDA>& m2, // to, model
    rm::MemoryView<rm::Matrix3x3, rm::VRAM_CUDA>& Cs,
    rm::MemoryView<unsigned int, rm::VRAM_CUDA>& Ncorr
    ) const
{
    // rm::StopWatch sw;
    // double el;

    // sw();

    size_t scanSize = m_width * m_height;
    // size_t scanSize = m_model_ram->width * m_model_ram->height;
    size_t Nrays = Tbm.size() * scanSize;

    rm::Memory<unsigned int,    rm::VRAM_CUDA> corr_valid(Nrays);
    rm::Memory<rm::Vector, rm::VRAM_CUDA> model_points(Nrays);
    rm::Memory<rm::Vector, rm::VRAM_CUDA> dataset_points(Nrays);

    rm::Memory<PinholeCorrectionDataRW, rm::RAM> mem(1);
    mem->model = m_model.raw();
    mem->ranges = m_ranges.raw();
    mem->Tsb = m_Tsb.raw();
    mem->Tbm = Tbm.raw();
    mem->Nposes = Tbm.size();
    mem->params = m_params.raw();
    mem->optical = m_optical;
    mem->handle = m_map->scene()->as()->handle;
    mem->corr_valid = corr_valid.raw();
    mem->model_points = model_points.raw();
    mem->dataset_points = dataset_points.raw();

    rm::Memory<PinholeCorrectionDataRW, rm::VRAM_CUDA> d_mem(1);
    copy(mem, d_mem, m_stream);


    rm::PipelinePtr program = make_pipeline_corr_rw(m_map->scene(), 1);


    // sw();
    RM_OPTIX_CHECK( optixLaunch(
        program->pipeline, 
        m_stream->handle(), 
        reinterpret_cast<CUdeviceptr>(d_mem.raw()), 
        sizeof( PinholeCorrectionDataRW ), 
        program->sbt,
        m_width, // width Xdim
        m_height, // height Ydim
        Tbm.size()// depth Zdim
        ));

    m_stream->synchronize();
    
    rm::sumBatched(corr_valid, Ncorr);
    meanBatched(dataset_points, corr_valid, Ncorr, m1);
    meanBatched(model_points, corr_valid, Ncorr, m2);

    covFancyBatched(model_points, m2, 
            dataset_points, m1, 
            corr_valid, Ncorr, Cs);
}

void PinholeCorrectorOptix::computeMeansCovsSW(
    const rm::MemoryView<rm::Transform, rm::VRAM_CUDA>& Tbm,
    rm::MemoryView<rm::Vector, rm::VRAM_CUDA>& m1,
    rm::MemoryView<rm::Vector, rm::VRAM_CUDA>& m2,
    rm::MemoryView<rm::Matrix3x3, rm::VRAM_CUDA>& Cs,
    rm::MemoryView<unsigned int, rm::VRAM_CUDA>& Ncorr
    ) const
{
    // Create Optix Memory structure
    rm::Memory<PinholeCorrectionDataSW, rm::RAM_CUDA> mem(1);
    mem->model = m_model.raw();
    mem->ranges = m_ranges.raw();
    mem->Tbm = Tbm.raw();
    mem->Tsb = m_Tsb.raw();
    mem->Nposes = Tbm.size();
    mem->params = m_params.raw();
    mem->optical = m_optical;
    mem->handle = m_map->scene()->as()->handle;
    mem->C = Cs.raw();
    mem->m1 = m1.raw(); // Sim
    mem->m2 = m2.raw(); // Real
    mem->Ncorr = Ncorr.raw();

    rm::Memory<PinholeCorrectionDataSW, rm::VRAM_CUDA> d_mem(1);
    copy(mem, d_mem, m_stream);

    rm::PipelinePtr program = make_pipeline_corr_sw(m_map->scene(), 1);

    RM_OPTIX_CHECK( optixLaunch(
        program->pipeline, 
        m_stream->handle(), 
        reinterpret_cast<CUdeviceptr>(d_mem.raw()), 
        sizeof( PinholeCorrectionDataSW ), 
        program->sbt, 
        Tbm.size(),
        1,
        1
        ));

    m_stream->synchronize();
}

} // namespace rmcl