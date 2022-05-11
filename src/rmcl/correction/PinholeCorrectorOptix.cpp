#include "rmcl/correction/PinholeCorrectorOptix.hpp"

#include "rmcl/correction/optix/PinholeCorrectProgramRW.hpp"
#include "rmcl/correction/optix/PinholeCorrectProgramSW.hpp"

#include "rmcl/correction/optix/CorrectionDataOptix.hpp"

#include <rmagine/math/math_batched.cuh>

#include <rmcl/math/math_batched.cuh>


// DEBUG
#include <rmagine/util/prints.h>
#include <rmagine/util/StopWatch.hpp>


namespace rm = rmagine;

namespace rmcl 
{

PinholeCorrectorOptix::PinholeCorrectorOptix(
    rmagine::OptixMapPtr map)
:Base(map)
{
    programs.resize(2);
    programs[0].reset(new PinholeCorrectProgramRW(map));
    programs[1].reset(new PinholeCorrectProgramSW(map));

    // CUDA_CHECK( cudaStreamCreate(&m_stream) );
    m_svd.reset(new rm::SVDCuda(Base::m_stream));

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
    // std::cout << "Start correction." << std::endl;
    CorrectionResults<rm::VRAM_CUDA> res;
    res.Ncorr.resize(Tbms.size());
    res.Tdelta.resize(Tbms.size());

    if(Tbms.size() == 0)
    {
        return res;
    }

    size_t scanSize = m_width * m_height;
    size_t Nrays = Tbms.size() * scanSize;

    // compute required additional bytes
    // except of res
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

    size_t required_bytes_per_ray_rw = 0;
    // corr_valid
    required_bytes_per_ray_rw += sizeof(unsigned int);
    // model points
    required_bytes_per_ray_rw += sizeof(rm::Vector);
    // dataset points
    required_bytes_per_ray_rw += sizeof(rm::Vector);


    size_t required_bytes_per_ray_sw = 0;

    // std::cout << "Model:" << std::endl;
    // std::cout << "- poses: " << Tbms.size() << std::endl;
    // std::cout << "- size: " << m_width * m_height << std::endl;
    // std::cout << "- rays: " << Nrays << std::endl;
    // std::cout << "Additional bytes required:" << std::endl;
    // std::cout << "- per pose: " << required_bytes_per_pose << ", total: " << required_bytes_per_pose * Tbms.size() << std::endl;
    // std::cout << "- RW. per ray: " << required_bytes_per_ray_rw << ", per pose: " << required_bytes_per_ray_rw * scanSize << std::endl;
    // std::cout << "- SW. per ray: " << required_bytes_per_ray_sw << ", per pose: " << required_bytes_per_ray_sw * scanSize << std::endl;
    
    size_t free_bytes, total_bytes;
    cudaMemGetInfo(&free_bytes, &total_bytes);

    size_t max_poses_rw = free_bytes / (required_bytes_per_pose + required_bytes_per_ray_rw * scanSize);

    max_poses_rw *= 3;
    max_poses_rw /= 4;

    // std::cout << "Max poses to compute " << std::endl;
    // std::cout << max_poses_rw << std::endl;
    // std::cout << "- RW: " << free_bytes / (required_bytes_per_pose + required_bytes_per_ray_rw * scanSize) << std::endl;
    // std::cout << "- SW: " << free_bytes / (required_bytes_per_pose + required_bytes_per_ray_sw * scanSize) << std::endl;

    rm::Memory<rm::Vector, rm::VRAM_CUDA> m1(Tbms.size());
    rm::Memory<rm::Vector, rm::VRAM_CUDA> m2(Tbms.size());
    rm::Memory<rm::Matrix3x3, rm::VRAM_CUDA> Cs(Tbms.size());
    rm::Memory<rm::Matrix3x3, rm::VRAM_CUDA> Us(Cs.size());
    rm::Memory<rm::Matrix3x3, rm::VRAM_CUDA> Vs(Cs.size());
    

    // what we need to decide what to do
    // - maximum allowed memory to use (default: max available?)
    // - performance switch when to use RW or SW

    // TODO how to make this dynamic somehow
    constexpr unsigned int POSE_SWITCH = 1024 * 8;
    // constexpr unsigned int POSE_SWITCH = 0;

    rm::StopWatch sw;
    double el;

    if(Tbms.size() > POSE_SWITCH)
    {
        // scanwise parallelization
        computeMeansCovsSW(Tbms, m1, m2, Cs, res.Ncorr);
    } else {
        
        if(Tbms.size() < max_poses_rw)
        {
            computeMeansCovsRW(Tbms, m1, m2, Cs, res.Ncorr);
        } else {
            // split
            std::cout << "Need to split!" << std::endl;
            
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
                auto m1_ = m1(chunk_start, chunk_end);
                auto m2_ = m2(chunk_start, chunk_end);
                auto Cs_ = Cs(chunk_start, chunk_end);
                auto Ncorr_ = res.Ncorr(chunk_start, chunk_end);

                computeMeansCovsRW(Tbms_, m1_, m2_, Cs_, Ncorr_);
            }
        }
    }

    m_svd->calcUV(Cs, Us, Vs);
    
    auto Rs = rm::multNxN(Us, rm::transpose(Vs));
    auto ts = rm::subNxN(m1, rm::multNxN(Rs, m2));
    rm::pack(Rs, ts, res.Tdelta);

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
    mem->handle = m_map->as.handle;
    mem->corr_valid = corr_valid.raw();
    mem->model_points = model_points.raw();
    mem->dataset_points = dataset_points.raw();

    rm::Memory<PinholeCorrectionDataRW, rm::VRAM_CUDA> d_mem(1);
    copy(mem, d_mem, Base::m_stream);

    rm::OptixProgramPtr program = programs[0];

    // el = sw();
    // std::cout << "- prepare: " << el << "s" << std::endl;

    // sw();
    OPTIX_CHECK( optixLaunch(
        program->pipeline, 
        Base::m_stream, 
        reinterpret_cast<CUdeviceptr>(d_mem.raw()), 
        sizeof( PinholeCorrectionDataRW ), 
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
    mem->handle = m_map->as.handle;
    mem->C = Cs.raw();
    mem->m1 = m1.raw(); // Sim
    mem->m2 = m2.raw(); // Real
    mem->Ncorr = Ncorr.raw();

    rm::Memory<PinholeCorrectionDataSW, rm::VRAM_CUDA> d_mem(1);
    copy(mem, d_mem, m_stream);

    rm::OptixProgramPtr program = programs[1];

    OPTIX_CHECK( optixLaunch(
        program->pipeline, 
        m_stream, 
        reinterpret_cast<CUdeviceptr>(d_mem.raw()), 
        sizeof( PinholeCorrectionDataSW ), 
        &program->sbt, 
        Tbm.size(),
        1,
        1
        ));

    cudaStreamSynchronize(m_stream);
}

} // namespace rmcl