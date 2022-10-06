#include <optix.h>
#include "rmcl/correction/optix/CorrectionDataOptix.hpp"
#include <rmagine/math/types.h>
#include <rmagine/map/optix/optix_sbt.h>

namespace rm = rmagine;

extern "C" {
__constant__ rmcl::OnDnCorrectionDataSW mem;
}

extern "C" __global__ void __raygen__rg()
{
    const float dist_thresh = mem.params->max_distance;

    // Lookup our location within the launch grid
    const uint3 idx = optixGetLaunchIndex();
    const uint3 dim = optixGetLaunchDimensions();

    const unsigned int Nvertical = mem.model->getHeight();
    const unsigned int Nhorizontal = mem.model->getWidth();

    const unsigned int pid = idx.z * dim.y * dim.x + idx.y * dim.x + idx.x;

    if(pid < mem.Nposes)
    {
        const float rangeMin = mem.model->range.min;
        const float rangeMax = mem.model->range.max;
        
        const rmagine::Transform Tsb = mem.Tsb[0];
        const rmagine::Transform Tbm = mem.Tbm[pid];
        const rmagine::Transform Tsm = Tbm * Tsb;
        const rmagine::Transform Tmb = Tbm.inv();

        // TODO: is it possible to not doing optixTrace twice?
        rmagine::Vector Dmean = {0.0, 0.0, 0.0};
        rmagine::Vector Mmean = {0.0, 0.0, 0.0};
        unsigned int Ncorr = 0;
        rmagine::Matrix3x3 C;
        C.setZeros();
        
        // Detecting Means
        for(unsigned int vid = 0; vid < Nvertical; vid++)
        {
            for(unsigned int hid = 0; hid < Nhorizontal; hid++)
            {
                // ids of results
                const unsigned int loc_id = mem.model->getBufferId(vid, hid);
                
                const float real_range = mem.ranges[loc_id];
                if(real_range < rangeMin || real_range > rangeMax){continue;}

                // Origin
                const rmagine::Vector ray_orig_s = mem.model->getOrigin(vid, hid);
                const rmagine::Vector ray_orig_b = Tsb * ray_orig_s;
                const rmagine::Vector ray_orig_m = Tsm * ray_orig_s;

                // Direction
                const rmagine::Vector ray_dir_s = mem.model->getDirection(vid, hid);
                const rmagine::Vector ray_dir_b = Tsb.R * ray_dir_s;
                const rmagine::Vector ray_dir_m = Tsm.R * ray_dir_s;

                unsigned int p0, p1, p2, p3;
                optixTrace(
                        mem.handle,
                        make_float3(ray_orig_m.x, ray_orig_m.y, ray_orig_m.z),
                        make_float3(ray_dir_m.x, ray_dir_m.y, ray_dir_m.z),
                        0.0f,               // Min intersection distance
                        rangeMax,                   // Max intersection distance
                        0.0f,                       // rayTime -- used for motion blur
                        OptixVisibilityMask( 1 ),   // Specify always visible
                        OPTIX_RAY_FLAG_DISABLE_ANYHIT,
                        0,          // SBT offset
                        1,             // SBT stride
                        0,          // missSBTIndex
                        p0, p1, p2, p3);

                const float range = __uint_as_float( p0 );
                if(range > rangeMax)
                {
                    continue;
                }

                const rmagine::Vector preal_b = ray_orig_b + ray_dir_b * real_range;
                const rmagine::Vector pint_b = ray_orig_b + ray_dir_b * range;

                const rmagine::Vector nint_m = {
                    __uint_as_float( p1 ),
                    __uint_as_float( p2 ),
                    __uint_as_float( p3 )
                };

                rmagine::Vector nint_b = Tmb.R * nint_m;
                
                if(nint_b.dot(ray_dir_b) > 0.0 )
                {
                    nint_b *= -1.0;
                }

                const float signed_plane_dist = (preal_b - pint_b).dot(nint_b);
                const rmagine::Vector pmesh_b = preal_b + nint_b * signed_plane_dist;

                const float dist_sqrt = (pmesh_b - preal_b).l2normSquared();

                if(dist_sqrt < dist_thresh * dist_thresh)
                {
                    Ncorr++;
                    Dmean += preal_b;
                    Mmean += pmesh_b;
                    C += preal_b.multT(pmesh_b);
                }
            }
        }

        mem.Ncorr[pid] = Ncorr;


        if(Ncorr > 0)
        {
            Dmean /= static_cast<float>(Ncorr);
            Mmean /= static_cast<float>(Ncorr);
            C /= static_cast<float>(Ncorr);
        }

        mem.C[pid] = C;
        mem.m1[pid] = Dmean;
        mem.m2[pid] = Mmean;
    }
}