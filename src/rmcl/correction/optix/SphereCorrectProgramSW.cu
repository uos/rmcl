#include <optix.h>
#include "mamcl/correction/optix/OptixCorrectionData.hpp"
#include "mamcl/math/math_cuda.hpp"
#include <Eigen/Dense>

extern "C" {
__constant__ mamcl::OptixCorrectionDataSW mem;
}

extern "C" __global__ void __raygen__rg()
{
    const float dist_thresh = mem.params->max_distance;

    // Lookup our location within the launch grid
    const uint3 idx = optixGetLaunchIndex();
    const uint3 dim = optixGetLaunchDimensions();

    const unsigned int Nhorizontal = mem.model->width;
    const unsigned int Nvertical = mem.model->height;

    const unsigned int pid = idx.z * dim.y * dim.x + idx.y * dim.x + idx.x;

    if(pid < mem.Nposes)
    {
        const float phiMin = mem.model->phi_min;
        const float thetaMin = mem.model->theta_min;
        const float rangeMin = mem.model->range_min;
        const float rangeMax = mem.model->range_max;
        
        const float hinc = (mem.model->theta_max - mem.model->theta_min) / ( static_cast<float>(Nhorizontal - 1) );
        const float vinc = (mem.model->phi_max - mem.model->phi_min) / ( static_cast<float>(Nvertical - 1) );
        
        const Eigen::Affine3f Tbm = mem.Tbm[pid];
        const Eigen::Affine3f Tsb = mem.Tsb[0];
        const Eigen::Affine3f Tsm = Tbm * Tsb;
        const Eigen::Affine3f Tmb = Tbm.inverse();

        // TODO: is it possible to not doing optixTrace twice?
        Eigen::Vector3f from_mean(0.0, 0.0, 0.0);
        Eigen::Vector3f to_mean(0.0, 0.0, 0.0);
        unsigned int Ncorr;
        Ncorr = 0;
        
        // Detecting Means
        for(unsigned int vid = 0; vid < Nvertical; vid++)
        {
            for(unsigned int hid = 0; hid < Nhorizontal; hid++)
            {
                // ids of results
                const unsigned int loc_id = vid * Nhorizontal + hid;
                
                const float real_range = mem.ranges[loc_id];
                if(real_range < rangeMin || real_range > rangeMax){continue;}

                const float theta = thetaMin + static_cast<float>(hid) * hinc;
                const float phi = phiMin + static_cast<float>(vid) * vinc;
                
                const Eigen::Vector3f ray_dir_s(
                        cos(phi) * cos(theta), 
                        cos(phi) * sin(theta), 
                        sin(phi));

                const Eigen::Vector3f ray_dir_b = Tsb.linear() * ray_dir_s;
                const Eigen::Vector3f ray_dir_m = Tsm.linear() * ray_dir_s;

                unsigned int p0, p1, p2, p3;
                optixTrace(
                        mem.handle,
                        make_float3(Tsm.translation().x(), Tsm.translation().y(), Tsm.translation().z()),
                        make_float3(ray_dir_m.x(), ray_dir_m.y(), ray_dir_m.z()),
                        0.0f,               // Min intersection distance
                        rangeMax,                   // Max intersection distance
                        0.0f,                       // rayTime -- used for motion blur
                        OptixVisibilityMask( 1 ),   // Specify always visible
                        OPTIX_RAY_FLAG_DISABLE_ANYHIT,
                        0,          // SBT offset
                        1,             // SBT stride
                        0,          // missSBTIndex
                        p0, p1, p2, p3);

                const float range = int_as_float( p0 );
                if(range > rangeMax)
                {
                    continue;
                }

                const Eigen::Vector3f preal_b = ray_dir_b * real_range;
                const Eigen::Vector3f pint_b = ray_dir_b * range;

                Eigen::Vector3f nint_b = Tmb.linear() * Eigen::Vector3f(
                        int_as_float( p1 ),
                        int_as_float( p2 ),
                        int_as_float( p3 )
                    );
                
                if(nint_b.dot(ray_dir_b) > 0.0 )
                {
                    nint_b *= -1.0;
                }

                const float signed_plane_dist = (preal_b - pint_b).dot(nint_b);
                const Eigen::Vector3f pmesh_b = preal_b + nint_b * signed_plane_dist;

                const float dist_sqrt = (pmesh_b - preal_b).squaredNorm();

                if(dist_sqrt < dist_thresh * dist_thresh)
                {
                    Ncorr++;
                    from_mean = from_mean + preal_b;
                    to_mean = to_mean + pmesh_b;
                }
            }
        }

        mem.Ncorr[pid] = Ncorr;

        Eigen::Matrix3f C;
        C.setZero();

        if(Ncorr > 0)
        {
            from_mean = from_mean / static_cast<float>(Ncorr);
            to_mean = to_mean / static_cast<float>(Ncorr);

            for(unsigned int vid = 0; vid < Nvertical; vid++)
            {
                for(unsigned int hid = 0; hid < Nhorizontal; hid++)
                {
                    // ids of results
                    const unsigned int loc_id = vid * Nhorizontal + hid;
                    
                    const float real_range = mem.ranges[loc_id];
                    if(real_range < rangeMin || real_range > rangeMax){continue;}

                    const float theta = thetaMin + static_cast<float>(hid) * hinc;
                    const float phi = phiMin + static_cast<double>(vid) * vinc;
                    
                    const Eigen::Vector3f ray_dir_s(
                            cos(phi) * cos(theta), 
                            cos(phi) * sin(theta), 
                            sin(phi));

                    const Eigen::Vector3f ray_dir_b = Tsb.linear() * ray_dir_s;
                    const Eigen::Vector3f ray_dir_m = Tsm.linear() * ray_dir_s;

                    unsigned int p0, p1, p2, p3;
                    optixTrace(
                            mem.handle,
                            make_float3(Tsm.translation().x(), Tsm.translation().y(), Tsm.translation().z()),
                            make_float3(ray_dir_m.x(), ray_dir_m.y(), ray_dir_m.z()),
                            0.0f,               // Min intersection distance
                            rangeMax,                   // Max intersection distance
                            0.0f,                       // rayTime -- used for motion blur
                            OptixVisibilityMask( 1 ),   // Specify always visible
                            OPTIX_RAY_FLAG_DISABLE_ANYHIT,
                            0,          // SBT offset
                            1,             // SBT stride
                            0,          // missSBTIndex
                            p0, p1, p2, p3);

                    const float range = int_as_float( p0 );
                    if(range > rangeMax)
                    {
                        continue;
                    }

                    const Eigen::Vector3f preal_b = ray_dir_b * real_range;
                    const Eigen::Vector3f pint_b = ray_dir_b * range;

                    Eigen::Vector3f nint_b = Tmb.linear() * Eigen::Vector3f(
                            int_as_float( p1 ),
                            int_as_float( p2 ),
                            int_as_float( p3 )
                        );
                    if(nint_b.dot(ray_dir_b) > 0.0 )
                    {
                        nint_b *= -1.0;
                    }

                    const float signed_plane_dist = (preal_b - pint_b).dot(nint_b);
                    const Eigen::Vector3f pmesh_b = preal_b + nint_b * signed_plane_dist;

                    const float dist_sqrt = (pmesh_b - preal_b).squaredNorm();

                    if(dist_sqrt < dist_thresh * dist_thresh)
                    {
                        const Eigen::Vector3f preal_centered = preal_b - from_mean;
                        const Eigen::Vector3f pmesh_centered = pmesh_b - to_mean;

                        C += preal_centered * pmesh_centered.transpose();
                    }
                }
            }

            C /= static_cast<float>(Ncorr);
        }

        mem.C[pid] = C;
        mem.m1[pid] = from_mean;
        mem.m2[pid] = to_mean;
    }
}

extern "C" __global__ void __miss__ms()
{
    optixSetPayload_0( float_as_int( mem.model->range_max + 1.0f ) );
}

extern "C" __global__ void __closesthit__ch()
{
    const float t = optixGetRayTmax();
    unsigned int normal_id = optixGetPrimitiveIndex();
    mamcl::HitGroupDataNormals* hg_data  = reinterpret_cast<mamcl::HitGroupDataNormals*>( optixGetSbtDataPointer() );

    optixSetPayload_0( float_as_int( t ) );
    optixSetPayload_1( float_as_int( hg_data->normals[normal_id].x ) );
    optixSetPayload_2( float_as_int( hg_data->normals[normal_id].y ) );
    optixSetPayload_3( float_as_int( hg_data->normals[normal_id].z ) );
}