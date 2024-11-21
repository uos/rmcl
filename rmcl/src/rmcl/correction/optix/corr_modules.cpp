#include "rmcl/correction/optix/corr_modules.h"
#include <rmagine/map/optix/OptixScene.hpp>
#include <optix.h>

namespace rm = rmagine;

namespace rmcl
{

// cache per (traversableGraphFlags ,scene)
std::unordered_map<unsigned int, 
    std::unordered_map<unsigned int, rm::ProgramModulePtr> 
> corr_module_cache_sw;


rm::ProgramModulePtr make_program_module_corr_sw(
    rm::OptixScenePtr scene,
    unsigned int sensor_id)
{
    unsigned int traversable_graph_flags = scene->traversableGraphFlags();


    auto scene_it = corr_module_cache_sw.find(traversable_graph_flags);

    if(scene_it != corr_module_cache_sw.end())
    {
        // found scene in cache
        auto sensor_it = scene_it->second.find(sensor_id);
        if(sensor_it != scene_it->second.end())
        {
            return sensor_it->second;
        }
    } else {
        // scene not found in cache -> creating empty new. filled later
        corr_module_cache_sw[traversable_graph_flags] = {};
    }



    rm::ProgramModulePtr ret = std::make_shared<rm::ProgramModule>();

    ret->compile_options->maxRegisterCount     = OPTIX_COMPILE_DEFAULT_MAX_REGISTER_COUNT;

    #ifndef NDEBUG
        ret->compile_options->optLevel             = OPTIX_COMPILE_OPTIMIZATION_LEVEL_0;
        ret->compile_options->debugLevel           = OPTIX_COMPILE_DEBUG_LEVEL_FULL;
    #else
        ret->compile_options->optLevel             = OPTIX_COMPILE_OPTIMIZATION_DEFAULT;
        #if OPTIX_VERSION >= 70400
        ret->compile_options->debugLevel           = OPTIX_COMPILE_DEBUG_LEVEL_MINIMAL;
        #else
        ret->compile_options->debugLevel           = OPTIX_COMPILE_DEBUG_LEVEL_NONE;
        #endif
    #endif

    if(sensor_id == 0)
    {
        static const char* kernel =
        #include "kernels/SphereCorrectProgramSWString.h"
        ;
        ret->ptx = std::string(kernel);
    } else if(sensor_id == 1) {
        const char *kernel =
        #include "kernels/PinholeCorrectProgramSWString.h"
        ;
        ret->ptx = std::string(kernel);
    } else if(sensor_id == 2) {
        const char *kernel =
        #include "kernels/O1DnCorrectProgramSWString.h"
        ;
        ret->ptx = std::string(kernel);
    } else if(sensor_id == 3) {
        const char *kernel =
        #include "kernels/OnDnCorrectProgramSWString.h"
        ;
        ret->ptx = std::string(kernel);
    } else {
        std::cout << "[OptixScene::raygen_ptx_from_model_type] ERROR model_type " << sensor_id << " not supported!" << std::endl;
        throw std::runtime_error("[OptixScene::raygen_ptx_from_model_type] ERROR loading ptx");
    }

    // TODO: share this between nearly any module?
    // depends on:
    // - scene : traversableGraphFlags
    // - numPayloadValues: modules/shader
    // - numAttributeValues: modules/shader
    // is used in:
    // - optixModuleCreateFromPTX
    // - optixPipelineCreate
    OptixPipelineCompileOptions pipeline_compile_options = {};
    {
        pipeline_compile_options.usesMotionBlur        = false;


        pipeline_compile_options.traversableGraphFlags = traversable_graph_flags;
        
        // max payload values: 32
        pipeline_compile_options.numPayloadValues      = 4;
        pipeline_compile_options.numAttributeValues    = 2;
    #ifndef NDEBUG // Enables debug exceptions during optix launches. This may incur significant performance cost and should only be done during development.
        pipeline_compile_options.exceptionFlags = OPTIX_EXCEPTION_FLAG_DEBUG | OPTIX_EXCEPTION_FLAG_TRACE_DEPTH | OPTIX_EXCEPTION_FLAG_STACK_OVERFLOW;
    #else
        pipeline_compile_options.exceptionFlags = OPTIX_EXCEPTION_FLAG_NONE;
    #endif
        pipeline_compile_options.pipelineLaunchParamsVariableName = "mem";
        pipeline_compile_options.usesPrimitiveTypeFlags = OPTIX_PRIMITIVE_TYPE_FLAGS_TRIANGLE;
    }

    ret->compile(&pipeline_compile_options, scene->context());

    // cache
    corr_module_cache_sw[traversable_graph_flags][sensor_id] = ret; 

    return ret;
}


// cache per (traversableGraphFlags ,scene)
std::unordered_map<unsigned int, 
    std::unordered_map<unsigned int, rm::ProgramModulePtr> 
> corr_module_cache_rw;


rmagine::ProgramModulePtr make_program_module_corr_rw(
    rmagine::OptixScenePtr scene,
    unsigned int sensor_id)
{
    unsigned int traversable_graph_flags = scene->traversableGraphFlags();


    auto scene_it = corr_module_cache_rw.find(traversable_graph_flags);

    if(scene_it != corr_module_cache_rw.end())
    {
        // found scene in cache
        auto sensor_it = scene_it->second.find(sensor_id);
        if(sensor_it != scene_it->second.end())
        {
            return sensor_it->second;
        }
    } else {
        // scene not found in cache -> creating empty new. filled later
        corr_module_cache_rw[traversable_graph_flags] = {};
    }



    rm::ProgramModulePtr ret = std::make_shared<rm::ProgramModule>();

    ret->compile_options->maxRegisterCount     = OPTIX_COMPILE_DEFAULT_MAX_REGISTER_COUNT;

    #ifndef NDEBUG
        ret->compile_options->optLevel             = OPTIX_COMPILE_OPTIMIZATION_LEVEL_0;
        ret->compile_options->debugLevel           = OPTIX_COMPILE_DEBUG_LEVEL_FULL;
    #else
        ret->compile_options->optLevel             = OPTIX_COMPILE_OPTIMIZATION_DEFAULT;
        #if OPTIX_VERSION >= 70400
        ret->compile_options->debugLevel           = OPTIX_COMPILE_DEBUG_LEVEL_MINIMAL;
        #else
        ret->compile_options->debugLevel           = OPTIX_COMPILE_DEBUG_LEVEL_NONE;
        #endif
    #endif


    if(sensor_id == 0)
    {
        static const char* kernel =
        #include "kernels/SphereCorrectProgramRWString.h"
        ;
        ret->ptx = std::string(kernel);
    } else if(sensor_id == 1) {
        const char *kernel =
        #include "kernels/PinholeCorrectProgramRWString.h"
        ;
        ret->ptx = std::string(kernel);
    } else if(sensor_id == 2) {
        const char *kernel =
        #include "kernels/O1DnCorrectProgramRWString.h"
        ;
        ret->ptx = std::string(kernel);
    } else if(sensor_id == 3) {
        const char *kernel =
        #include "kernels/OnDnCorrectProgramRWString.h"
        ;
        ret->ptx = std::string(kernel);
    } else {
        std::cout << "[OptixScene::raygen_ptx_from_model_type] ERROR model_type " << sensor_id << " not supported!" << std::endl;
        throw std::runtime_error("[OptixScene::raygen_ptx_from_model_type] ERROR loading ptx");
    }


    // TODO: share this between nearly any module?
    // depends on:
    // - scene : traversableGraphFlags
    // - numPayloadValues: modules/shader
    // - numAttributeValues: modules/shader
    // is used in:
    // - optixModuleCreateFromPTX
    // - optixPipelineCreate
    OptixPipelineCompileOptions pipeline_compile_options = {};
    {
        pipeline_compile_options.usesMotionBlur        = false;


        pipeline_compile_options.traversableGraphFlags = traversable_graph_flags;
        
        // max payload values: 32
        pipeline_compile_options.numPayloadValues      = 4;
        pipeline_compile_options.numAttributeValues    = 2;
    #ifndef NDEBUG // Enables debug exceptions during optix launches. This may incur significant performance cost and should only be done during development.
        pipeline_compile_options.exceptionFlags = OPTIX_EXCEPTION_FLAG_DEBUG | OPTIX_EXCEPTION_FLAG_TRACE_DEPTH | OPTIX_EXCEPTION_FLAG_STACK_OVERFLOW;
    #else
        pipeline_compile_options.exceptionFlags = OPTIX_EXCEPTION_FLAG_NONE;
    #endif
        pipeline_compile_options.pipelineLaunchParamsVariableName = "mem";
        pipeline_compile_options.usesPrimitiveTypeFlags = OPTIX_PRIMITIVE_TYPE_FLAGS_TRIANGLE;
    }

    ret->compile(&pipeline_compile_options, scene->context());

    // cache
    corr_module_cache_rw[traversable_graph_flags][sensor_id] = ret; 

    return ret;
}

} // namespace rmcl