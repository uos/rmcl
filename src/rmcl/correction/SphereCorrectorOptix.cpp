#include "rmcl/correction/SphereCorrectorOptix.hpp"


#include "rmcl/correction/optix/corr_pipelines.h"

#include "rmcl/correction/optix/CorrectionDataOptix.hpp"

#include <rmagine/math/math_batched.cuh>

#include <rmcl/math/math_batched.cuh>
#include <rmcl/math/math.cuh>

#include <optix_stubs.h>

#include <rmagine/util/StopWatch.hpp>



// DEBUG
#include <rmagine/util/prints.h>


namespace rm = rmagine;

namespace rmcl 
{

SphereCorrectorOptix::SphereCorrectorOptix()
:Base()
{
    m_svd = std::make_shared<rm::SVDCuda>(Base::m_stream);

    // default params
    CorrectionParams params;
    setParams(params);
}

SphereCorrectorOptix::SphereCorrectorOptix(
    rmagine::OptixMapPtr map)
:Base(map)
{
    m_svd = std::make_shared<rm::SVDCuda>(Base::m_stream);

    // default params
    CorrectionParams params;
    setParams(params);
}

void SphereCorrectorOptix::setParams(
    const CorrectionParams& params)
{
    rm::Memory<CorrectionParams, rm::RAM> params_ram(1);
    params_ram[0] = params;
    m_params = params_ram;
}

void SphereCorrectorOptix::setInputData(
    const rm::MemoryView<float, rmagine::VRAM_CUDA>& ranges)
{
    m_ranges = ranges;
}

CorrectionResults<rm::VRAM_CUDA> SphereCorrectorOptix::correct(
    const rm::MemoryView<rm::Transform, rm::VRAM_CUDA>& Tbms) const
{
    if(!m_stream->context()->isActive())
    {
        std::cout << "[SphereCorrectorOptix::correct() Need to activate map context" << std::endl;
        m_stream->context()->use();
    }

    CorrectionResults<rm::VRAM_CUDA> res;
    if(Tbms.size() == 0)
    {
        return res;
    }

    res.Ncorr.resize(Tbms.size());
    res.Tdelta.resize(Tbms.size());

    rm::Memory<rm::Vector, rm::VRAM_CUDA> ds(Tbms.size());
    rm::Memory<rm::Vector, rm::VRAM_CUDA> ms(Tbms.size());
    rm::Memory<rm::Matrix3x3, rm::VRAM_CUDA> Cs(Tbms.size());
    rm::Memory<rm::Matrix3x3, rm::VRAM_CUDA> Us(Cs.size());
    rm::Memory<rm::Matrix3x3, rm::VRAM_CUDA> Vs(Cs.size());

    computeCovs(Tbms, ds, ms, Cs, res.Ncorr);

    static CorrectionCuda corr(m_svd);
    corr.correction_from_covs(ds, ms, Cs, res.Ncorr, res.Tdelta);

    return res;
}

void SphereCorrectorOptix::computeCovs(
    const rm::MemoryView<rm::Transform, rm::VRAM_CUDA>& Tbms,
    rm::MemoryView<rm::Vector, rm::VRAM_CUDA>& ds,
    rm::MemoryView<rm::Vector, rm::VRAM_CUDA>& ms,
    rm::MemoryView<rm::Matrix3x3, rm::VRAM_CUDA>& Cs,
    rm::MemoryView<unsigned int, rm::VRAM_CUDA>& Ncorr) const
{
    // TODO how to make this dynamic somehow
    constexpr unsigned int POSE_SWITCH = 1024 * 8;
    // constexpr unsigned int POSE_SWITCH = 1024 * 0;

    if(Tbms.size() > POSE_SWITCH)
    {
        // scanwise parallelization
        computeMeansCovsSW(Tbms, ds, ms, Cs, Ncorr);
    } else {
        // raywise parallelization
        computeMeansCovsRW(Tbms, ds, ms, Cs, Ncorr);
    }
}

void SphereCorrectorOptix::computeCovs(
    const rmagine::MemoryView<rmagine::Transform, rmagine::VRAM_CUDA>& Tbms,
    CorrectionPreResults<rmagine::VRAM_CUDA>& res
) const
{
    if(!m_stream->context()->isActive())
    {
        std::cout << "[SphereCorrectorOptix::computeCovs() Need to activate map context" << std::endl;
        m_stream->context()->use();
    }
    computeCovs(Tbms, res.ds, res.ms, res.Cs, res.Ncorr);
}

CorrectionPreResults<rmagine::VRAM_CUDA> SphereCorrectorOptix::computeCovs(
    const rmagine::MemoryView<rmagine::Transform, rmagine::VRAM_CUDA>& Tbms
) const
{
    CorrectionPreResults<rmagine::VRAM_CUDA> res;
    res.ds.resize(Tbms.size());
    res.ms.resize(Tbms.size());
    res.Cs.resize(Tbms.size());
    res.Ncorr.resize(Tbms.size());
    computeCovs(Tbms, res);
    return res;
}


void SphereCorrectorOptix::findSPC(
    const rm::MemoryView<rm::Transform, rm::VRAM_CUDA>& Tbms,
    rm::Memory<rm::Point, rm::VRAM_CUDA>& dataset_points,
    rm::Memory<rm::Point, rm::VRAM_CUDA>& model_points,
    rm::Memory<unsigned int, rm::VRAM_CUDA>& corr_valid
) const
{
    if(!m_stream->context()->isActive())
    {
        std::cout << "[SphereCorrectorOptix::findSPC() Need to activate map context" << std::endl;
        m_stream->context()->use();
    }

    size_t scanSize = m_width * m_height;
    size_t Nrays = Tbms.size() * scanSize;

    // keep the this if bigger than required
    if(dataset_points.size() < Nrays)
    {
        dataset_points.resize(Nrays);
    }

    if(model_points.size() < Nrays)
    {
        model_points.resize(Nrays);
    }

    if(corr_valid.size() < Nrays)
    {
        corr_valid.resize(Nrays);
    }

    findSPC(Tbms, dataset_points(0, Nrays), model_points(0, Nrays), corr_valid(0, Nrays) );
}

void SphereCorrectorOptix::findSPC(
    const rm::MemoryView<rm::Transform, rm::VRAM_CUDA>& Tbms,
    rm::MemoryView<rm::Point, rm::VRAM_CUDA> dataset_points,
    rm::MemoryView<rm::Point, rm::VRAM_CUDA> model_points,
    rm::MemoryView<unsigned int, rm::VRAM_CUDA> corr_valid
) const
{
    // std::cout << "SphereCorrectorOptix - findSPC" << std::endl;
    if(!m_map)
    {
        throw std::runtime_error("NO MAP");
    }

    if(!m_map->scene())
    {
        throw std::runtime_error("EMPTY MAP");
    }

    if(!m_map->scene()->as())
    {
        throw std::runtime_error("MAP SCENE NOT COMMITTED");
    }

    if(!m_stream->context()->isActive())
    {
        std::cout << "[SphereCorrectorOptix::findSPC() Need to activate map context" << std::endl;
        m_stream->context()->use();
    }

    rm::Memory<SphereCorrectionDataRW, rm::RAM> mem(1);
    mem->model = m_model.raw();
    mem->ranges = m_ranges.raw();
    mem->Tsb = m_Tsb.raw();
    mem->Tbm = Tbms.raw();
    mem->Nposes = Tbms.size();
    mem->params = m_params.raw();
    mem->handle = m_map->scene()->as()->handle;
    mem->corr_valid = corr_valid.raw();
    mem->dataset_points = dataset_points.raw();
    mem->model_points = model_points.raw();

    rm::Memory<SphereCorrectionDataRW, rm::VRAM_CUDA> d_mem(1);
    copy(mem, d_mem, m_stream);

    rm::PipelinePtr program = make_pipeline_corr_rw(m_map->scene(), 0);

    RM_OPTIX_CHECK( optixLaunch(
        program->pipeline, 
        m_stream->handle(), 
        reinterpret_cast<CUdeviceptr>(d_mem.raw()), 
        sizeof( SphereCorrectionDataRW ), 
        program->sbt,
        m_width, // width Xdim
        m_height, // height Ydim
        Tbms.size()// depth Zdim
        ));

    m_stream->synchronize();
}

/// PRIVATE
void SphereCorrectorOptix::computeMeansCovsRW(
    const rm::MemoryView<rm::Transform, rm::VRAM_CUDA>& Tbm,
    rm::MemoryView<rm::Vector, rm::VRAM_CUDA>& means_dataset, // from, dataset
    rm::MemoryView<rm::Vector, rm::VRAM_CUDA>& means_model, // to, model
    rm::MemoryView<rm::Matrix3x3, rm::VRAM_CUDA>& Cs,
    rm::MemoryView<unsigned int, rm::VRAM_CUDA>& Ncorr
    ) const
{
    size_t scanSize = m_width * m_height;
    size_t Nrays = Tbm.size() * scanSize;

    rm::Memory<rm::Vector, rm::VRAM_CUDA> dataset_points(Nrays);
    rm::Memory<rm::Vector, rm::VRAM_CUDA> model_points(Nrays);
    rm::Memory<unsigned int,    rm::VRAM_CUDA> corr_valid(Nrays);

    findSPC(Tbm, dataset_points, model_points, corr_valid);

    // one pass function: TODO test

    // new: one-pass
    means_covs_online_batched(
        dataset_points, model_points, corr_valid, // input
        means_dataset, means_model, // outputs
        Cs, Ncorr
    );
}

void SphereCorrectorOptix::computeMeansCovsSW(
    const rm::MemoryView<rm::Transform, rm::VRAM_CUDA>& Tbm,
    rm::MemoryView<rm::Vector, rm::VRAM_CUDA>& means_dataset, // from, dataset
    rm::MemoryView<rm::Vector, rm::VRAM_CUDA>& means_model, // to, model
    rm::MemoryView<rm::Matrix3x3, rm::VRAM_CUDA>& Cs,
    rm::MemoryView<unsigned int, rm::VRAM_CUDA>& Ncorr
    ) const
{
    // Create Optix Memory structure
    rm::Memory<SphereCorrectionDataSW, rm::RAM_CUDA> mem(1);
    mem->model = m_model.raw();
    mem->ranges = m_ranges.raw();
    mem->Tbm = Tbm.raw();
    mem->Tsb = m_Tsb.raw();
    mem->Nposes = Tbm.size();
    mem->params = m_params.raw();
    mem->handle = m_map->scene()->as()->handle;
    mem->C = Cs.raw();
    mem->m1 = means_dataset.raw();
    mem->m2 = means_model.raw();
    mem->Ncorr = Ncorr.raw();

    rm::Memory<SphereCorrectionDataSW, rm::VRAM_CUDA> d_mem(1);
    copy(mem, d_mem, m_stream);

    rm::PipelinePtr program = make_pipeline_corr_sw(m_map->scene(), 0);
    
    RM_OPTIX_CHECK( optixLaunch(
        program->pipeline, 
        m_stream->handle(), 
        reinterpret_cast<CUdeviceptr>(d_mem.raw()), 
        sizeof( SphereCorrectionDataSW ), 
        program->sbt, 
        Tbm.size(),
        1,
        1
        ));

    m_stream->synchronize();
}



} // namespace rmcl