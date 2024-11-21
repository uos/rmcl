#include "rmcl/correction/optix/corr_program_groups.h"
#include "rmcl/correction/optix/corr_modules.h"

#include <rmagine/map/optix/OptixScene.hpp>
#include <rmagine/util/optix/OptixDebug.hpp>

#include <optix_stubs.h>

namespace rm = rmagine;

namespace rmcl
{

CorrRayGenProgramGroup::~CorrRayGenProgramGroup()
{
    if(record_h)
    {
        cudaFreeHost(record_h);
    }
}

CorrMissProgramGroup::~CorrMissProgramGroup()
{
    if(record_h)
    {
        cudaFreeHost(record_h);
    }
}

CorrHitProgramGroup::~CorrHitProgramGroup()
{
    if(record_h)
    {
        cudaFreeHost(record_h);
    }
}


void CorrHitProgramGroup::onSBTUpdated(
    bool size_changed)
{
    rm::OptixScenePtr scene = m_scene.lock();

    if(scene)
    {
        if(size_changed)
        {
            size_t n_hitgroups_required = scene->requiredSBTEntries();

            if(n_hitgroups_required > record_count)
            {
                // std::cout << "HitGroup update number of records: " << record_count << " -> " << n_hitgroups_required << std::endl;
                if(record_h)
                {
                    RM_CUDA_CHECK( cudaFreeHost( record_h ) );
                }
                
                RM_CUDA_CHECK( cudaMallocHost( &record_h, n_hitgroups_required * record_stride ) );

                for(size_t i=0; i<n_hitgroups_required; i++)
                {
                    RM_OPTIX_CHECK( optixSbtRecordPackHeader( prog_group, &record_h[i] ) );
                }

                if( record )
                {
                    RM_CUDA_CHECK( cudaFree( reinterpret_cast<void*>( record ) ) );
                }
                
                RM_CUDA_CHECK( cudaMalloc( reinterpret_cast<void**>( &record ), n_hitgroups_required * record_stride ) );

                record_count = n_hitgroups_required;
            }
        }

        for(size_t i=0; i<record_count; i++)
        {
            record_h[i].data = scene->sbt_data;
        }

        RM_CUDA_CHECK( cudaMemcpyAsync(
                    reinterpret_cast<void*>( record ),
                    record_h,
                    record_count * record_stride,
                    cudaMemcpyHostToDevice,
                    scene->stream()->handle()
                    ) );
    }
    
}

std::unordered_map<rm::OptixSceneWPtr, 
    std::unordered_map<rm::ProgramModuleWPtr, CorrRayGenProgramGroupPtr>
> m_program_group_corr_gen_sw_cache;

CorrRayGenProgramGroupPtr make_program_group_corr_gen_sw(
    rm::OptixScenePtr scene,
    rm::ProgramModulePtr module)
{
    auto scene_it = m_program_group_corr_gen_sw_cache.find(scene);

    if(scene_it != m_program_group_corr_gen_sw_cache.end())
    {
        auto module_it = scene_it->second.find(module);
        if(module_it != scene_it->second.end())
        {
            return module_it->second;
        }
    } else {
        m_program_group_corr_gen_sw_cache[scene] = {};
    }

    CorrRayGenProgramGroupPtr ret = std::make_shared<CorrRayGenProgramGroup>();

    // set description
    ret->description->kind = OPTIX_PROGRAM_GROUP_KIND_RAYGEN;
    ret->description->raygen.module = module->module;
    ret->description->raygen.entryFunctionName = "__raygen__rg";

    // set module
    ret->module = module;

    ret->create();


    { // init SBT Records
        const size_t raygen_record_size     = sizeof( CorrRayGenProgramGroup::SbtRecordData );
        
        RM_CUDA_CHECK( cudaMallocHost( 
            &ret->record_h, 
            raygen_record_size ) );

        RM_OPTIX_CHECK( optixSbtRecordPackHeader( 
            ret->prog_group,
            &ret->record_h[0] ) );

        RM_CUDA_CHECK( cudaMalloc( reinterpret_cast<void**>( &ret->record ), raygen_record_size ) );

        RM_CUDA_CHECK( cudaMemcpyAsync(
                    reinterpret_cast<void*>( ret->record ),
                    ret->record_h,
                    raygen_record_size,
                    cudaMemcpyHostToDevice,
                    scene->stream()->handle()
                    ) );

        ret->record_stride = sizeof( CorrRayGenProgramGroup::SbtRecordData );
        ret->record_count = 1;
    }

    // cache
    m_program_group_corr_gen_sw_cache[scene][module] = ret;

    return ret;
}

CorrRayGenProgramGroupPtr make_program_group_corr_gen_sw(
    rm::OptixScenePtr scene,
    unsigned int sensor_id)
{
    rm::ProgramModulePtr module = make_program_module_corr_sw(scene, sensor_id);
    return make_program_group_corr_gen_sw(scene, module);
}


std::unordered_map<rm::OptixSceneWPtr, 
    std::unordered_map<rm::ProgramModuleWPtr, CorrMissProgramGroupPtr>
> m_program_group_corr_miss_sw_cache;

CorrMissProgramGroupPtr make_program_group_corr_miss_sw(
    rm::OptixScenePtr scene,
    rm::ProgramModulePtr module)
{
    auto scene_it = m_program_group_corr_miss_sw_cache.find(scene);

    if(scene_it != m_program_group_corr_miss_sw_cache.end())
    {
        auto module_it = scene_it->second.find(module);
        if(module_it != scene_it->second.end())
        {
            return module_it->second;
        }
    } else {
        m_program_group_corr_miss_sw_cache[scene] = {};
    }

    CorrMissProgramGroupPtr ret = std::make_shared<CorrMissProgramGroup>();

    ret->description->kind                     = OPTIX_PROGRAM_GROUP_KIND_MISS;
    ret->description->raygen.module            = module->module;
    ret->description->raygen.entryFunctionName = "__miss__ms";

    #if OPTIX_VERSION >= 70400
    ret->options->payloadType = &module->compile_options->payloadTypes[0];
    #endif
    ret->module = module;

    ret->create();

    { // init SBT Records
        const size_t miss_record_size     = sizeof( CorrMissProgramGroup::SbtRecordData );
        
        RM_CUDA_CHECK( cudaMallocHost( 
            &ret->record_h, 
            miss_record_size ) );

        RM_OPTIX_CHECK( optixSbtRecordPackHeader( 
            ret->prog_group,
            &ret->record_h[0] ) );

        RM_CUDA_CHECK( cudaMalloc( reinterpret_cast<void**>( &ret->record ), miss_record_size ) );

        RM_CUDA_CHECK( cudaMemcpyAsync(
                    reinterpret_cast<void*>( ret->record ),
                    ret->record_h,
                    miss_record_size,
                    cudaMemcpyHostToDevice,
                    scene->stream()->handle()
                    ) );

        ret->record_stride = sizeof( CorrMissProgramGroup::SbtRecordData );
        ret->record_count = 1;
    }

    m_program_group_corr_miss_sw_cache[scene][module] = ret;

    return ret;
}


CorrMissProgramGroupPtr make_program_group_corr_miss_sw(
    rmagine::OptixScenePtr scene)
{
    // Test this. hard coded 0 should work
    rm::ProgramModulePtr module = make_program_module_corr_sw(scene, 0);
    return make_program_group_corr_miss_sw(scene, module);
}


std::unordered_map<rm::OptixSceneWPtr, 
    std::unordered_map<rm::ProgramModuleWPtr, CorrHitProgramGroupPtr>
> m_program_group_corr_hit_sw_cache;


CorrHitProgramGroupPtr make_program_group_corr_hit_sw(
    rm::OptixScenePtr scene,
    rm::ProgramModulePtr module)
{
    auto scene_it = m_program_group_corr_hit_sw_cache.find(scene);

    if(scene_it != m_program_group_corr_hit_sw_cache.end())
    {
        auto module_it = scene_it->second.find(module);
        if(module_it != scene_it->second.end())
        {
            return module_it->second;
        }
    } else {
        m_program_group_corr_hit_sw_cache[scene] = {};
    }

    CorrHitProgramGroupPtr ret = std::make_shared<CorrHitProgramGroup>();

    ret->description->kind                     = OPTIX_PROGRAM_GROUP_KIND_HITGROUP;
    ret->description->raygen.module            = module->module;
    ret->description->raygen.entryFunctionName = "__closesthit__ch";

    #if OPTIX_VERSION >= 70400
    ret->options->payloadType = &module->compile_options->payloadTypes[0];
    #endif
    ret->module = module;

    ret->create();


    { // init SBT Records
        const size_t n_hitgroup_records = scene->requiredSBTEntries();   
        const size_t hitgroup_record_size     = sizeof( CorrHitProgramGroup::SbtRecordData ) * n_hitgroup_records;
        
        RM_CUDA_CHECK( cudaMallocHost( 
            &ret->record_h, 
            hitgroup_record_size ) );

        for(size_t i=0; i<n_hitgroup_records; i++)
        {
            RM_OPTIX_CHECK( optixSbtRecordPackHeader( 
                ret->prog_group,
                &ret->record_h[i] ) );
            ret->record_h[i].data = scene->sbt_data;
        }
        
        RM_CUDA_CHECK( cudaMalloc( reinterpret_cast<void**>( &ret->record ), hitgroup_record_size ) );

        RM_CUDA_CHECK( cudaMemcpyAsync(
                    reinterpret_cast<void*>( ret->record ),
                    ret->record_h,
                    hitgroup_record_size,
                    cudaMemcpyHostToDevice,
                    scene->stream()->handle()
                    ) );

        ret->record_stride = sizeof( CorrHitProgramGroup::SbtRecordData );
        ret->record_count = n_hitgroup_records;
    }

    scene->addEventReceiver(ret);

    m_program_group_corr_hit_sw_cache[scene][module] = ret;

    return ret;
}

CorrHitProgramGroupPtr make_program_group_corr_hit_sw(
    rmagine::OptixScenePtr scene)
{
    rm::ProgramModulePtr module = make_program_module_corr_sw(scene, 0);
    return make_program_group_corr_hit_sw(scene, module);
}



std::unordered_map<rm::OptixSceneWPtr, 
    std::unordered_map<rm::ProgramModuleWPtr, CorrRayGenProgramGroupPtr>
> m_program_group_corr_gen_rw_cache;

CorrRayGenProgramGroupPtr make_program_group_corr_gen_rw(
    rm::OptixScenePtr scene,
    rm::ProgramModulePtr module)
{
    auto scene_it = m_program_group_corr_gen_rw_cache.find(scene);

    if(scene_it != m_program_group_corr_gen_rw_cache.end())
    {
        auto module_it = scene_it->second.find(module);
        if(module_it != scene_it->second.end())
        {
            return module_it->second;
        }
    } else {
        m_program_group_corr_gen_rw_cache[scene] = {};
    }

    CorrRayGenProgramGroupPtr ret = std::make_shared<CorrRayGenProgramGroup>();

    // set description
    ret->description->kind = OPTIX_PROGRAM_GROUP_KIND_RAYGEN;
    ret->description->raygen.module = module->module;
    ret->description->raygen.entryFunctionName = "__raygen__rg";

    // set module
    ret->module = module;

    ret->create();


    { // init SBT Records
        const size_t raygen_record_size     = sizeof( CorrRayGenProgramGroup::SbtRecordData );
        
        RM_CUDA_CHECK( cudaMallocHost( 
            &ret->record_h, 
            raygen_record_size ) );

        RM_OPTIX_CHECK( optixSbtRecordPackHeader( 
            ret->prog_group,
            &ret->record_h[0] ) );

        RM_CUDA_CHECK( cudaMalloc( reinterpret_cast<void**>( &ret->record ), raygen_record_size ) );

        RM_CUDA_CHECK( cudaMemcpyAsync(
                    reinterpret_cast<void*>( ret->record ),
                    ret->record_h,
                    raygen_record_size,
                    cudaMemcpyHostToDevice,
                    scene->stream()->handle()
                    ) );

        ret->record_stride = sizeof( CorrRayGenProgramGroup::SbtRecordData );
        ret->record_count = 1;
    }

    // cache
    m_program_group_corr_gen_rw_cache[scene][module] = ret;

    return ret;
}

CorrRayGenProgramGroupPtr make_program_group_corr_gen_rw(
    rm::OptixScenePtr scene,
    unsigned int sensor_id)
{
    rm::ProgramModulePtr module = make_program_module_corr_rw(scene, sensor_id);
    return make_program_group_corr_gen_rw(scene, module);
}


std::unordered_map<rm::OptixSceneWPtr, 
    std::unordered_map<rm::ProgramModuleWPtr, CorrMissProgramGroupPtr>
> m_program_group_corr_miss_rw_cache;

CorrMissProgramGroupPtr make_program_group_corr_miss_rw(
    rm::OptixScenePtr scene,
    rm::ProgramModulePtr module)
{
    auto scene_it = m_program_group_corr_miss_rw_cache.find(scene);

    if(scene_it != m_program_group_corr_miss_rw_cache.end())
    {
        auto module_it = scene_it->second.find(module);
        if(module_it != scene_it->second.end())
        {
            return module_it->second;
        }
    } else {
        m_program_group_corr_miss_rw_cache[scene] = {};
    }

    CorrMissProgramGroupPtr ret = std::make_shared<CorrMissProgramGroup>();

    ret->description->kind                     = OPTIX_PROGRAM_GROUP_KIND_MISS;
    ret->description->raygen.module            = module->module;
    ret->description->raygen.entryFunctionName = "__miss__ms";

    #if OPTIX_VERSION >= 70400
    ret->options->payloadType = &module->compile_options->payloadTypes[0];
    #endif
    ret->module = module;

    ret->create();

    { // init SBT Records
        const size_t miss_record_size     = sizeof( CorrMissProgramGroup::SbtRecordData );
        
        RM_CUDA_CHECK( cudaMallocHost( 
            &ret->record_h, 
            miss_record_size ) );

        RM_OPTIX_CHECK( optixSbtRecordPackHeader( 
            ret->prog_group,
            &ret->record_h[0] ) );

        RM_CUDA_CHECK( cudaMalloc( reinterpret_cast<void**>( &ret->record ), miss_record_size ) );

        RM_CUDA_CHECK( cudaMemcpyAsync(
                    reinterpret_cast<void*>( ret->record ),
                    ret->record_h,
                    miss_record_size,
                    cudaMemcpyHostToDevice,
                    scene->stream()->handle()
                    ) );

        ret->record_stride = sizeof( CorrMissProgramGroup::SbtRecordData );
        ret->record_count = 1;
    }

    m_program_group_corr_miss_rw_cache[scene][module] = ret;

    return ret;
}


CorrMissProgramGroupPtr make_program_group_corr_miss_rw(
    rmagine::OptixScenePtr scene)
{
    // Test this. hard coded 0 should work
    rm::ProgramModulePtr module = make_program_module_corr_rw(scene, 0);
    return make_program_group_corr_miss_rw(scene, module);
}


std::unordered_map<rm::OptixSceneWPtr, 
    std::unordered_map<rm::ProgramModuleWPtr, CorrHitProgramGroupPtr>
> m_program_group_corr_hit_rw_cache;


CorrHitProgramGroupPtr make_program_group_corr_hit_rw(
    rm::OptixScenePtr scene,
    rm::ProgramModulePtr module)
{
    auto scene_it = m_program_group_corr_hit_rw_cache.find(scene);

    if(scene_it != m_program_group_corr_hit_rw_cache.end())
    {
        auto module_it = scene_it->second.find(module);
        if(module_it != scene_it->second.end())
        {
            return module_it->second;
        }
    } else {
        m_program_group_corr_hit_rw_cache[scene] = {};
    }

    CorrHitProgramGroupPtr ret = std::make_shared<CorrHitProgramGroup>();

    ret->description->kind                     = OPTIX_PROGRAM_GROUP_KIND_HITGROUP;
    ret->description->raygen.module            = module->module;
    ret->description->raygen.entryFunctionName = "__closesthit__ch";

    #if OPTIX_VERSION >= 70400
    ret->options->payloadType = &module->compile_options->payloadTypes[0];
    #endif
    ret->module = module;

    ret->create();


    { // init SBT Records
        const size_t n_hitgroup_records = scene->requiredSBTEntries();   
        const size_t hitgroup_record_size     = sizeof( CorrHitProgramGroup::SbtRecordData ) * n_hitgroup_records;
        
        RM_CUDA_CHECK( cudaMallocHost( 
            &ret->record_h, 
            hitgroup_record_size ) );

        for(size_t i=0; i<n_hitgroup_records; i++)
        {
            RM_OPTIX_CHECK( optixSbtRecordPackHeader( 
                ret->prog_group,
                &ret->record_h[i] ) );
            ret->record_h[i].data = scene->sbt_data;
        }
        
        RM_CUDA_CHECK( cudaMalloc( reinterpret_cast<void**>( &ret->record ), hitgroup_record_size ) );

        RM_CUDA_CHECK( cudaMemcpyAsync(
                    reinterpret_cast<void*>( ret->record ),
                    ret->record_h,
                    hitgroup_record_size,
                    cudaMemcpyHostToDevice,
                    scene->stream()->handle()
                    ) );

        ret->record_stride = sizeof( CorrHitProgramGroup::SbtRecordData );
        ret->record_count = n_hitgroup_records;
    }

    scene->addEventReceiver(ret);

    m_program_group_corr_hit_rw_cache[scene][module] = ret;

    return ret;
}

CorrHitProgramGroupPtr make_program_group_corr_hit_rw(
    rmagine::OptixScenePtr scene)
{
    rm::ProgramModulePtr module = make_program_module_corr_rw(scene, 0);
    return make_program_group_corr_hit_rw(scene, module);
}


} // namespace rmcl