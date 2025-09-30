#include <optix.h>
#include "rmcl_ros/rmcl/optix/EvaluationDataOptix.hpp"
#include <rmagine/math/types.h>
#include <rmagine/map/optix/optix_sbt.h>

#include <rmcl_ros/rmcl/optix/cuda_math_helper.cuh>
#include <rmcl_ros/rmcl/optix/cuda_rmagine_conversions.cuh>

namespace rm = rmagine;

extern "C" {
__constant__ rmcl::BeamEvaluationData mem;
}

extern "C" __global__ void __raygen__rg()
{
    // const float dist_thresh = mem.params->max_distance;

    // Lookup our location within the launch grid
    const uint3 idx = optixGetLaunchIndex();
    const uint3 dim = optixGetLaunchDimensions();

    // const unsigned int Nvertical = mem.model->getHeight();
    // const unsigned int Nhorizontal = mem.model->getWidth();

    const float sigma_dist = mem.dist_sigma;
    const float sigma_dist_quad = sigma_dist * sigma_dist;

    const bool real_hit = mem.sensor_range.inside(mem.meas.range);

    const unsigned int pid = idx.z * dim.y * dim.x + idx.y * dim.x + idx.x;

    if(pid < mem.Nparticles)
    {
        // TODO
        const rm::Transform Tbm = mem.particle_poses[pid];
        const rm::Transform Tsm = Tbm * mem.Tsb;

        const rmcl::RangeMeasurement meas_m = Tsm * mem.meas;
        
        // optixSetPayload_0(real_hit);

        unsigned int p0;
        optixTrace(
                mem.handle,
                make_float3(meas_m.orig.x, meas_m.orig.y, meas_m.orig.z), // orig world
                make_float3(meas_m.dir.x,  meas_m.dir.y,  meas_m.dir.z), // dir world
                0.0f,                       // Min intersection distance
                10000.0,                    // Max intersection distance
                0.0f,                       // rayTime -- used for motion blur
                OptixVisibilityMask( 1 ),   // Specify always visible
                OPTIX_RAY_FLAG_DISABLE_ANYHIT,
                0,          // SBT offset
                1,             // SBT stride
                0,          // missSBTIndex
                p0);

        const float error = __uint_as_float(p0);

        const float eval = exp(-(error * error) / sigma_dist_quad / 2) / (sqrt(2 * sigma_dist_quad * M_PI));
        // const float eval_eval_eval = eval * eval * eval;

        rmcl::ParticleAttributes particle_attr_new = mem.particle_attrs[pid];

        rm::Gaussian1D meas;
        meas.mean = eval;
        meas.sigma = 0.0;
        meas.n_meas = 1;

        particle_attr_new.likelihood += meas;
        particle_attr_new.likelihood.n_meas = min(particle_attr_new.likelihood.n_meas, 10000);

        mem.particle_attrs[pid] = particle_attr_new;
    }
}

extern "C" __global__ void __closesthit__ch()
{
    const bool real_hit = mem.sensor_range.inside(mem.meas.range);

    if(real_hit)
    {
        // sim hit + real hit
        const float3 ray_orig_m = optixGetWorldRayOrigin();
        const float3 ray_dir_m = optixGetWorldRayDirection();
        const float range = optixGetRayTmax();

        const unsigned int face_id = optixGetPrimitiveIndex();
        const unsigned int inst_id = optixGetInstanceId();
        const unsigned int gas_id = optixGetSbtGASIndex();

        rm::OptixSceneSBT* scene_data  = reinterpret_cast<rm::OptixSceneSBT*>(optixGetSbtDataPointer());

        rm::OptixMeshSBT* mesh_data = nullptr;
        if(scene_data->type == rm::OptixSceneType::INSTANCES)
        {
            // instance hierarchy
            rm::OptixSceneSBT* inst_scene = scene_data->geometries[inst_id].inst_data.scene;
            mesh_data = &(inst_scene->geometries[gas_id].mesh_data);
        } else {
            mesh_data = &scene_data->geometries[gas_id].mesh_data;
        }

        const float3 nint_o = make_float3(
            mesh_data->face_normals[face_id].x, 
            mesh_data->face_normals[face_id].y, 
            mesh_data->face_normals[face_id].z);
        const float3 nint_m = optixTransformNormalFromObjectToWorldSpace(nint_o);
        const float3 pint_m = ray_orig_m + ray_dir_m * range;
        const float3 preal_m = ray_orig_m + ray_dir_m * mem.meas.range;

        const float signed_plane_dist = dot(pint_m - preal_m, nint_m);
        const float plane_dist = abs(signed_plane_dist);

        optixSetPayload_0(__float_as_uint(plane_dist)); // error
    } else {
        // hit in simulation but not in reality -> bad
        optixSetPayload_0(__float_as_uint(mem.real_miss_sim_hit_error));
    }
}

extern "C" __global__ void __miss__ms()
{
    const bool real_hit = mem.sensor_range.inside(mem.meas.range);
    if(real_hit)
    {
        optixSetPayload_0(__float_as_uint(mem.real_hit_sim_miss_error)); // bad
    } else {
        optixSetPayload_0(__float_as_uint(mem.real_miss_sim_miss_error)); // good
    }
}