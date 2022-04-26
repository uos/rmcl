#include "mamcl/correction/optix/ScanCorrectProgramSW.hpp"
#include "mamcl/correction/optix/OptixCorrectionData.hpp"

#include "mamcl/util/GenericAlign.hpp"
#include "mamcl/util/optix/OptixDebug.hpp"

#include <optix_stubs.h>


#include "mamcl/util/optix/OptixUtil.hpp"
#include "mamcl/util/optix/OptixSbtRecord.hpp"
#include "mamcl/util/optix/OptixData.hpp"

#include <cuda_runtime.h>

#include <iostream>
#include <fstream>

namespace mamcl {

typedef SbtRecord<RayGenDataEmpty>     RayGenSbtRecord;
typedef SbtRecord<MissDataEmpty>       MissSbtRecord;
typedef SbtRecord<HitGroupDataNormals>   HitGroupSbtRecord;

ScanCorrectProgramSW::ScanCorrectProgramSW(OptixMeshPtr mesh)
{
    // 1. INIT MODULE

    char log[2048]; // For error reporting from OptiX creation functions
    size_t sizeof_log = sizeof( log );
    
    OptixModuleCompileOptions module_compile_options = {};
    module_compile_options.maxRegisterCount     = OPTIX_COMPILE_DEFAULT_MAX_REGISTER_COUNT;
#ifdef DEBUG
    module_compile_options.optLevel             = OPTIX_COMPILE_OPTIMIZATION_LEVEL_0;
    module_compile_options.debugLevel           = OPTIX_COMPILE_DEBUG_LEVEL_FULL;
#else
    module_compile_options.optLevel             = OPTIX_COMPILE_OPTIMIZATION_DEFAULT;
    module_compile_options.debugLevel           = OPTIX_COMPILE_DEBUG_LEVEL_NONE;
#endif


    OptixPipelineCompileOptions pipeline_compile_options = {};
    pipeline_compile_options.usesMotionBlur        = false;
    pipeline_compile_options.traversableGraphFlags = OPTIX_TRAVERSABLE_GRAPH_FLAG_ALLOW_SINGLE_GAS;
    pipeline_compile_options.numPayloadValues      = 4;
    pipeline_compile_options.numAttributeValues    = 0;
#ifdef DEBUG // Enables debug exceptions during optix launches. This may incur significant performance cost and should only be done during development.
    pipeline_compile_options.exceptionFlags = OPTIX_EXCEPTION_FLAG_DEBUG | OPTIX_EXCEPTION_FLAG_TRACE_DEPTH | OPTIX_EXCEPTION_FLAG_STACK_OVERFLOW;
#else
    pipeline_compile_options.exceptionFlags = OPTIX_EXCEPTION_FLAG_NONE;
#endif
    pipeline_compile_options.pipelineLaunchParamsVariableName = "mem";
    pipeline_compile_options.usesPrimitiveTypeFlags = OPTIX_PRIMITIVE_TYPE_FLAGS_TRIANGLE;

    std::string optix_ptx_dir(OPTIX_PTX_DIR);
    std::string optix_program_name = "ScanCorrectProgramSW";

    std::string filename = optix_ptx_dir + "/cuda_compile_ptx_1_generated_" + optix_program_name + ".cu.ptx";

    std::cout << "Load Program from " << filename << std::endl;

    std::string ptx;

    std::ifstream file( filename.c_str() );
    if( file.good() )
    {
        // Found usable source file
        std::stringstream source_buffer;
        source_buffer << file.rdbuf();
        ptx = source_buffer.str();
    }

    OPTIX_CHECK_LOG( optixModuleCreateFromPTX(
                mesh->context,
                &module_compile_options,
                &pipeline_compile_options,
                ptx.c_str(),
                ptx.size(),
                log,
                &sizeof_log,
                &module
                ) );

    // 2. initProgramGroups

    OptixProgramGroupOptions program_group_options   = {}; // Initialize to zeros

    OptixProgramGroupDesc raygen_prog_group_desc    = {}; //
    raygen_prog_group_desc.kind                     = OPTIX_PROGRAM_GROUP_KIND_RAYGEN;
    raygen_prog_group_desc.raygen.module            = module;
    raygen_prog_group_desc.raygen.entryFunctionName = "__raygen__rg";
    sizeof_log = sizeof( log );

    optixProgramGroupCreate(
                mesh->context,
                &raygen_prog_group_desc,
                1,   // num program groups
                &program_group_options,
                log,
                &sizeof_log,
                &raygen_prog_group
                );

    OptixProgramGroupDesc miss_prog_group_desc  = {};
    miss_prog_group_desc.kind                   = OPTIX_PROGRAM_GROUP_KIND_MISS;
    miss_prog_group_desc.miss.module            = module;
    miss_prog_group_desc.miss.entryFunctionName = "__miss__ms";
    sizeof_log = sizeof( log );
    optixProgramGroupCreate(
                mesh->context,
                &miss_prog_group_desc,
                1,   // num program groups
                &program_group_options,
                log,
                &sizeof_log,
                &miss_prog_group
                );

    OptixProgramGroupDesc hitgroup_prog_group_desc = {};
    hitgroup_prog_group_desc.kind                         = OPTIX_PROGRAM_GROUP_KIND_HITGROUP;
    hitgroup_prog_group_desc.hitgroup.moduleCH            = module;
    hitgroup_prog_group_desc.hitgroup.entryFunctionNameCH = "__closesthit__ch";
    sizeof_log = sizeof( log );
    optixProgramGroupCreate(
                mesh->context,
                &hitgroup_prog_group_desc,
                1,   // num program groups
                &program_group_options,
                log,
                &sizeof_log,
                &hitgroup_prog_group
                );

    // 3. link pipeline
    const uint32_t    max_trace_depth  = 1;
    OptixProgramGroup program_groups[] = { 
        raygen_prog_group, 
        miss_prog_group, 
        hitgroup_prog_group 
    };

    OptixPipelineLinkOptions pipeline_link_options = {};
    pipeline_link_options.maxTraceDepth          = max_trace_depth;
#ifdef DEBUG
    pipeline_link_options.debugLevel             = OPTIX_COMPILE_DEBUG_LEVEL_FULL;
#else
    pipeline_link_options.debugLevel             = OPTIX_COMPILE_DEBUG_LEVEL_DEFAULT;
#endif
    sizeof_log = sizeof( log );
    OPTIX_CHECK_LOG( optixPipelineCreate(
                mesh->context,
                &pipeline_compile_options,
                &pipeline_link_options,
                program_groups,
                sizeof( program_groups ) / sizeof( program_groups[0] ),
                log,
                &sizeof_log,
                &pipeline
                ) );

    OptixStackSizes stack_sizes = {};
    for( auto& prog_group : program_groups )
    {
        OPTIX_CHECK( optixUtilAccumulateStackSizes( prog_group, &stack_sizes ) );
    }

    uint32_t direct_callable_stack_size_from_traversal;
    uint32_t direct_callable_stack_size_from_state;
    uint32_t continuation_stack_size;
    OPTIX_CHECK( optixUtilComputeStackSizes( &stack_sizes, max_trace_depth,
                                                0,  // maxCCDepth
                                                0,  // maxDCDEpth
                                                &direct_callable_stack_size_from_traversal,
                                                &direct_callable_stack_size_from_state, &continuation_stack_size ) );
    OPTIX_CHECK( optixPipelineSetStackSize( pipeline, direct_callable_stack_size_from_traversal,
                                            direct_callable_stack_size_from_state, continuation_stack_size,
                                            1  // maxTraversableDepth
                                            ) );

    // 4. setup shader binding table
    CUdeviceptr  raygen_record;
    const size_t raygen_record_size = sizeof( RayGenSbtRecord );
    CUDA_CHECK( cudaMalloc( reinterpret_cast<void**>( &raygen_record ), raygen_record_size ) );
    RayGenSbtRecord rg_sbt;
    OPTIX_CHECK( optixSbtRecordPackHeader( raygen_prog_group, &rg_sbt ) );
    CUDA_CHECK( cudaMemcpy(
                reinterpret_cast<void*>( raygen_record ),
                &rg_sbt,
                raygen_record_size,
                cudaMemcpyHostToDevice
                ) );

    CUdeviceptr miss_record;
    size_t      miss_record_size = sizeof( MissSbtRecord );
    CUDA_CHECK( cudaMalloc( reinterpret_cast<void**>( &miss_record ), miss_record_size ) );
    MissSbtRecord ms_sbt;
    
    OPTIX_CHECK( optixSbtRecordPackHeader( miss_prog_group, &ms_sbt ) );
    CUDA_CHECK( cudaMemcpy(
                reinterpret_cast<void*>( miss_record ),
                &ms_sbt,
                miss_record_size,
                cudaMemcpyHostToDevice
                ) );

    CUdeviceptr hitgroup_record;
    size_t      hitgroup_record_size = sizeof( HitGroupSbtRecord );
    CUDA_CHECK( cudaMalloc( reinterpret_cast<void**>( &hitgroup_record ), hitgroup_record_size ) );
    HitGroupSbtRecord hg_sbt;
    OPTIX_CHECK( optixSbtRecordPackHeader( hitgroup_prog_group, &hg_sbt ) );
    hg_sbt.data.normals = mesh->normals.raw();

    CUDA_CHECK( cudaMemcpy(
                reinterpret_cast<void*>( hitgroup_record ),
                &hg_sbt,
                hitgroup_record_size,
                cudaMemcpyHostToDevice
                ) );

    sbt.raygenRecord                = raygen_record;
    sbt.missRecordBase              = miss_record;
    sbt.missRecordStrideInBytes     = sizeof( MissSbtRecord );
    sbt.missRecordCount             = 1;
    sbt.hitgroupRecordBase          = hitgroup_record;
    sbt.hitgroupRecordStrideInBytes = sizeof( HitGroupSbtRecord );
    sbt.hitgroupRecordCount         = 1;
}

} // namespace mamcl