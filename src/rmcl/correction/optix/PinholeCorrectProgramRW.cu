#include <optix.h>
#include "rmcl/correction/optix/CorrectionDataOptix.hpp"
#include <rmagine/math/types.h>
#include <rmagine/util/optix/OptixData.hpp>

extern "C" {
__constant__ rmcl::PinholeCorrectionDataRW mem;
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

    // if(hid == 0 && vid == 0 && pid == 0)
    // {
    //     printf("PinholeCorrectProgramRW.cu\n");
    // }

    const unsigned int loc_id = mem.model->getBufferId(vid, hid);
    const unsigned int glob_id = pid * mem.model->size() + loc_id;

    const rmagine::Transform Tsb = mem.Tsb[0];
    const rmagine::Transform Tbm = mem.Tbm[pid];
    const rmagine::Transform Tsm = Tbm * Tsb;
    const rmagine::Quaternion Rmb = Tbm.R.inv();
    
    rmagine::Vector ray_dir_s;
    if(mem.optical)
    {
        ray_dir_s = mem.model->getDirectionOptical(vid, hid);
    } else {
        ray_dir_s = mem.model->getDirection(vid, hid);
    }        

    const rmagine::Vector ray_dir_b = Tsb.R * ray_dir_s;
    const rmagine::Vector ray_dir_m = Tsm.R * ray_dir_s;

    unsigned int p0, p1, p2, p3;
    optixTrace(
            mem.handle,
            make_float3(Tsm.t.x, Tsm.t.y, Tsm.t.z ),
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
        mem.corr_valid[glob_id] = 0;
        mem.model_points[glob_id] = {0.0, 0.0, 0.0};
        mem.dataset_points[glob_id] = {0.0, 0.0, 0.0};
        return;
    }
        
    const rmagine::Vector preal_b = ray_dir_b * real_range;
    const rmagine::Vector psim_b = ray_dir_b * sim_range;
    
    const rmagine::Vector nsim_m = {
        __uint_as_float(p1),
        __uint_as_float(p2),
        __uint_as_float(p3)
    };

    rmagine::Vector nsim_b = Rmb * nsim_m;

    if(ray_dir_b.dot(nsim_b) > 0.0)
    {
        nsim_b = -nsim_b;
    }

    const float signed_plane_dist = (preal_b - psim_b).dot(nsim_b);
    const rmagine::Vector pnearest_b = preal_b + nsim_b * signed_plane_dist;
    const float dist_sqrt = (pnearest_b - preal_b).l2normSquared();

    if(dist_sqrt < dist_thresh * dist_thresh)
    {
        mem.corr_valid[glob_id] = 1;
        mem.model_points[glob_id] = pnearest_b;
        mem.dataset_points[glob_id] = preal_b;
    } else {
        mem.corr_valid[glob_id] = 0;
    }
}

extern "C" __global__ void __miss__ms()
{
    optixSetPayload_0( __float_as_uint( mem.model->range.max + 1.0f ) );
}

extern "C" __global__ void __closesthit__ch()
{
    const float t = optixGetRayTmax();

    // Get additional info
    const unsigned int face_id = optixGetPrimitiveIndex();
    const unsigned int object_id = optixGetInstanceIndex();

    rmagine::HitGroupDataNormals* hg_data  = reinterpret_cast<rmagine::HitGroupDataNormals*>( optixGetSbtDataPointer() );

    const rmagine::Vector normal_rm = hg_data->normals[object_id][face_id];
    float3 normal = make_float3(normal_rm.x, normal_rm.y, normal_rm.z);
    float3 normal_world = optixTransformNormalFromObjectToWorldSpace(normal);

    optixSetPayload_0( __float_as_uint( t ) );
    optixSetPayload_1( __float_as_uint( normal_world.x ) );
    optixSetPayload_2( __float_as_uint( normal_world.y ) );
    optixSetPayload_3( __float_as_uint( normal_world.z ) );
}