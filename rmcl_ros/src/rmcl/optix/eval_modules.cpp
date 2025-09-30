#include "rmcl_ros/rmcl/optix/eval_modules.h"
#include <rmagine/map/optix/OptixScene.hpp>
#include <optix.h>

namespace rm = rmagine;

namespace rmcl
{

// cache per (traversableGraphFlags ,scene)
std::unordered_map<unsigned int, rm::ProgramModulePtr> eval_module_cache;

rm::ProgramModulePtr make_program_module_eval(
    rm::OptixScenePtr scene)
{
    unsigned int traversable_graph_flags = scene->traversableGraphFlags();


    auto scene_it = eval_module_cache.find(traversable_graph_flags);

    if(scene_it != eval_module_cache.end())
    {
      // found scene in cache
      return scene_it->second;
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

    static const char* kernel =
    #include "kernels/BeamEvaluateProgramString.h"
    ;
    ret->ptx = std::string(kernel);

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
    eval_module_cache[traversable_graph_flags] = ret;

    return ret;
}

} // namespace rmcl