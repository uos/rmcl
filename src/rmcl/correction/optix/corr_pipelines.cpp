#include "rmcl/correction/optix/corr_pipelines.h"
#include "rmcl/correction/optix/corr_program_groups.h"

#include <optix_stubs.h>
#include <cuda_runtime.h>
#include <rmagine/map/optix/OptixScene.hpp>
#include <rmagine/simulation/optix/common.h>

namespace rm = rmagine;

namespace rmcl
{


CorrPipeline::~CorrPipeline()
{
    // std::cout << "[SimPipeline::~SimPipeline()] destroyed." << std::endl;
}

void CorrPipeline::onDepthChanged()
{
    // std::cout << "[SimPipeline::onDepthChanged]" << std::endl;
    // TODO
}


void CorrPipeline::onCommitDone(
    const rm::OptixSceneCommitResult& info)
{
    // std::cout << "[SimPipeline::onCommitDone]" << std::endl;
    
    // connect changed sbt data
    if(prog_groups.size() > 0)
    {
        rm::ProgramGroupPtr hit = prog_groups[2];
        sbt->hitgroupRecordBase          = hit->record;
        sbt->hitgroupRecordStrideInBytes = hit->record_stride;
        sbt->hitgroupRecordCount         = hit->record_count;
    }
}


std::unordered_map<rm::OptixSceneWPtr,
    std::unordered_map<unsigned int, CorrPipelinePtr>
> pipeline_corr_sw_cache;

CorrPipelinePtr make_pipeline_corr_sw(
    rm::OptixScenePtr scene,
    unsigned int sensor_id)
{
    auto scene_it = pipeline_corr_sw_cache.find(scene);

    if(scene_it != pipeline_corr_sw_cache.end())
    {
        auto sensor_it = scene_it->second.find(sensor_id);
        if(sensor_it != scene_it->second.end())
        {
            return sensor_it->second;
        }
    } else {
        pipeline_corr_sw_cache[scene] = {};
    }

    unsigned int traversable_graph_flags = scene->traversableGraphFlags();

    CorrPipelinePtr ret = std::make_shared<CorrPipeline>();


    rm::ProgramGroupPtr raygen = make_program_group_corr_gen_sw(scene, sensor_id);
    rm::ProgramGroupPtr miss = make_program_group_corr_miss_sw(scene);
    rm::ProgramGroupPtr hit = make_program_group_corr_hit_sw(scene);

    ret->sbt->raygenRecord                = raygen->record;

    ret->sbt->missRecordBase              = miss->record;
    ret->sbt->missRecordStrideInBytes     = miss->record_stride;
    ret->sbt->missRecordCount             = miss->record_count;

    ret->sbt->hitgroupRecordBase          = hit->record;
    ret->sbt->hitgroupRecordStrideInBytes = hit->record_stride;
    ret->sbt->hitgroupRecordCount         = hit->record_count;

    uint32_t          max_traversable_depth = scene->depth();
    const uint32_t    max_trace_depth  = 1; // TODO: 31 is maximum. Set this dynamically?

    ret->prog_groups = {
        raygen,
        miss,
        hit
    };


    // LINK OPTIONS
    ret->link_options->maxTraceDepth          = max_trace_depth;

    #if OPTIX_VERSION < 70700
    #ifndef NDEBUG
        ret->link_options->debugLevel             = OPTIX_COMPILE_DEBUG_LEVEL_FULL;
    #else
        ret->link_options->debugLevel             = OPTIX_COMPILE_DEBUG_LEVEL_DEFAULT;
    #endif // NDEBUG
    #endif // VERSION

    { // COMPILE OPTIONS
        ret->compile_options->usesMotionBlur        = false;

        ret->compile_options->traversableGraphFlags = traversable_graph_flags;
        
        // max payload values: 32
        ret->compile_options->numPayloadValues      = 4;
        ret->compile_options->numAttributeValues    = 2;
    #ifndef NDEBUG // Enables debug exceptions during optix launches. This may incur significant performance cost and should only be done during development.
        ret->compile_options->exceptionFlags = OPTIX_EXCEPTION_FLAG_DEBUG | OPTIX_EXCEPTION_FLAG_TRACE_DEPTH | OPTIX_EXCEPTION_FLAG_STACK_OVERFLOW;
    #else
        ret->compile_options->exceptionFlags = OPTIX_EXCEPTION_FLAG_NONE;
    #endif
        ret->compile_options->pipelineLaunchParamsVariableName = "mem";
        ret->compile_options->usesPrimitiveTypeFlags = OPTIX_PRIMITIVE_TYPE_FLAGS_TRIANGLE;
    }

    ret->create(scene->context());

    scene->addEventReceiver(ret);

    // caching is important. we dont want to receive events from scene more than once per commit
    pipeline_corr_sw_cache[scene][sensor_id] = ret;

    return ret;
}


std::unordered_map<rm::OptixSceneWPtr,
    std::unordered_map<unsigned int, CorrPipelinePtr>
> pipeline_corr_rw_cache;

CorrPipelinePtr make_pipeline_corr_rw(
    rm::OptixScenePtr scene,
    unsigned int sensor_id)
{
    auto scene_it = pipeline_corr_rw_cache.find(scene);

    if(scene_it != pipeline_corr_rw_cache.end())
    {
        auto sensor_it = scene_it->second.find(sensor_id);
        if(sensor_it != scene_it->second.end())
        {
            return sensor_it->second;
        }
    } else {
        pipeline_corr_rw_cache[scene] = {};
    }

    unsigned int traversable_graph_flags = scene->traversableGraphFlags();

    CorrPipelinePtr ret = std::make_shared<CorrPipeline>();


    rm::ProgramGroupPtr raygen = make_program_group_corr_gen_rw(scene, sensor_id);
    rm::ProgramGroupPtr miss = make_program_group_corr_miss_rw(scene);
    rm::ProgramGroupPtr hit = make_program_group_corr_hit_rw(scene);

    ret->sbt->raygenRecord                = raygen->record;

    ret->sbt->missRecordBase              = miss->record;
    ret->sbt->missRecordStrideInBytes     = miss->record_stride;
    ret->sbt->missRecordCount             = miss->record_count;

    ret->sbt->hitgroupRecordBase          = hit->record;
    ret->sbt->hitgroupRecordStrideInBytes = hit->record_stride;
    ret->sbt->hitgroupRecordCount         = hit->record_count;

    uint32_t          max_traversable_depth = scene->depth();
    const uint32_t    max_trace_depth  = 1; // TODO: 31 is maximum. Set this dynamically?

    ret->prog_groups = {
        raygen,
        miss,
        hit
    };


    // LINK OPTIONS
    ret->link_options->maxTraceDepth          = max_trace_depth;

    #if OPTIX_VERSION < 70700
    #ifndef NDEBUG
        ret->link_options->debugLevel             = OPTIX_COMPILE_DEBUG_LEVEL_FULL;
    #else
        ret->link_options->debugLevel             = OPTIX_COMPILE_DEBUG_LEVEL_DEFAULT;
    #endif // NDEBUG
    #endif // OPTIX_VERSION

    { // COMPILE OPTIONS
        ret->compile_options->usesMotionBlur        = false;

        ret->compile_options->traversableGraphFlags = traversable_graph_flags;
        
        // max payload values: 32
        ret->compile_options->numPayloadValues      = 4;
        ret->compile_options->numAttributeValues    = 2;
    #ifndef NDEBUG // Enables debug exceptions during optix launches. This may incur significant performance cost and should only be done during development.
        ret->compile_options->exceptionFlags = OPTIX_EXCEPTION_FLAG_DEBUG | OPTIX_EXCEPTION_FLAG_TRACE_DEPTH | OPTIX_EXCEPTION_FLAG_STACK_OVERFLOW;
    #else
        ret->compile_options->exceptionFlags = OPTIX_EXCEPTION_FLAG_NONE;
    #endif
        ret->compile_options->pipelineLaunchParamsVariableName = "mem";
        ret->compile_options->usesPrimitiveTypeFlags = OPTIX_PRIMITIVE_TYPE_FLAGS_TRIANGLE;
    }

    ret->create(scene->context());

    scene->addEventReceiver(ret);

    // caching is important. we dont want to receive events from scene more than once per commit
    pipeline_corr_rw_cache[scene][sensor_id] = ret;

    return ret;
}


} // namespace rmcl