#include <optix.h>
#include "rmcl/correction/optix/CorrectionDataOptix.hpp"
#include <rmagine/math/types.h>
#include <rmagine/map/optix/optix_sbt.h>

namespace rm = rmagine;

extern "C" {
__constant__ rmcl::O1DnCorrectionDataRW mem;
}

extern "C" __global__ void __raygen__rg()
{
    // Parameters
    const float dist_thresh = mem.params->max_distance;
    const float range_max = mem.model->range.max;
    const float range_min = mem.model->range.min;

    const uint3 idx = optixGetLaunchIndex();
    const uint3 dim = optixGetLaunchDimensions();
    
    // vertical id
    const unsigned int hid = idx.x;
    // horizonatal id
    const unsigned int vid = idx.y;
    // pose id
    const unsigned int pid = idx.z;

    const unsigned int loc_id = mem.model->getBufferId(vid, hid);
    const unsigned int glob_id = pid * mem.model->size() + loc_id;

    const rm::Transform Tsb = mem.Tsb[0];
    const rm::Transform Tbm = mem.Tbm[pid];
    const rm::Transform Tsm = Tbm * Tsb;
    const rm::Quaternion Rms = Tsm.R.inv();
    
    // Origin
    const rm::Vector ray_orig_s = mem.model->getOrigin(vid, hid);
    const rm::Vector ray_orig_m = Tsm * ray_orig_s;

    // Direction
    const rm::Vector ray_dir_s = mem.model->getDirection(vid, hid);
    const rm::Vector ray_dir_m = Tsm.R * ray_dir_s;

    unsigned int p0, p1, p2, p3;
    optixTrace(
            mem.handle,
            make_float3(ray_orig_m.x, ray_orig_m.y, ray_orig_m.z ),
            make_float3(ray_dir_m.x, ray_dir_m.y, ray_dir_m.z),
            0.0f,                       // Min intersection distance
            range_max,                   // Max intersection distance
            0.0f,                       // rayTime -- used for motion blur
            OptixVisibilityMask( 1 ),   // Specify always visible
            OPTIX_RAY_FLAG_DISABLE_ANYHIT,
            0,          // SBT offset
            1,             // SBT stride
            0,          // missSBTIndex
            p0, p1, p2, p3 );

    const float sim_range = __uint_as_float( p0 );
    const float real_range = mem.ranges[loc_id];

    if(real_range > range_max || sim_range > range_max || real_range < range_min)
    {
        mem.dataset_points[glob_id] = {0.0f, 0.0f, 0.0f};
        mem.model_points[glob_id] = {0.0f, 0.0f, 0.0f};
        mem.corr_valid[glob_id] = 0;
    } else {
        rm::Vector nsim_m = {
            __uint_as_float(p1),
            __uint_as_float(p2),
            __uint_as_float(p3)
        };

        nsim_m.normalizeInplace();

        // going to sensor space
        const rm::Vector preal_s = ray_orig_s + ray_dir_s * real_range;
        const rm::Vector psim_s = ray_orig_s + ray_dir_s * sim_range;
        
        rm::Vector nsim_s = Rms * nsim_m;

        // if(ray_dir_s.dot(nsim_s) > 0.0)
        // {
        //     nsim_s = -nsim_s;
        // }

        const float signed_plane_dist = (psim_s - preal_s).dot(nsim_s);
        const rm::Vector pnearest_s = preal_s + nsim_s * signed_plane_dist;
        const float dist_sqrt = (pnearest_s - preal_s).l2normSquared();

        if(dist_sqrt < dist_thresh * dist_thresh)
        {
            mem.dataset_points[glob_id] = Tsb * preal_s;
            mem.model_points[glob_id] = Tsb * pnearest_s;
            mem.corr_valid[glob_id] = 1;
        } else {
            mem.dataset_points[glob_id] = {0.0f, 0.0f, 0.0f};
            mem.model_points[glob_id] = {0.0f, 0.0f, 0.0f};
            mem.corr_valid[glob_id] = 0;
        }
    }
}

extern "C" __global__ void __miss__ms()
{
    optixSetPayload_0( __float_as_uint( mem.model->range.max + 1.0f ) );
}

extern "C" __global__ void __closesthit__ch()
{
    const float t = optixGetRayTmax();
    const unsigned int face_id = optixGetPrimitiveIndex();
    const unsigned int inst_id = optixGetInstanceId();
    const unsigned int gas_id = optixGetSbtGASIndex();

    rm::OptixSceneSBT* scene_data  = reinterpret_cast<rm::OptixSceneSBT*>( optixGetSbtDataPointer() );

    rm::OptixMeshSBT* mesh_data = nullptr;
    if(scene_data->type == rm::OptixSceneType::INSTANCES)
    {
        // instance hierarchy
        rm::OptixSceneSBT* inst_scene = scene_data->geometries[inst_id].inst_data.scene;
        mesh_data = &(inst_scene->geometries[gas_id].mesh_data);
    } else {
        mesh_data = &scene_data->geometries[gas_id].mesh_data;
    }

    const float3 normal = make_float3(
        mesh_data->face_normals[face_id].x, 
        mesh_data->face_normals[face_id].y, 
        mesh_data->face_normals[face_id].z);
    float3 normal_world = optixTransformNormalFromObjectToWorldSpace(normal);

    optixSetPayload_0( __float_as_uint( t ) );
    optixSetPayload_1( __float_as_uint( normal_world.x ) );
    optixSetPayload_2( __float_as_uint( normal_world.y ) );
    optixSetPayload_3( __float_as_uint( normal_world.z ) );
}