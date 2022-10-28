#include <optix.h>
#include "rmcl/correction/optix/CorrectionDataOptix.hpp"
#include <rmagine/math/types.h>
#include <rmagine/map/optix/optix_sbt.h>

namespace rm = rmagine;

extern "C" {
__constant__ rmcl::O1DnCorrectionDataSW mem;
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
        
        const rm::Transform Tsb = mem.Tsb[0];
        const rm::Transform Tbm = mem.Tbm[pid];
        const rm::Transform Tsm = Tbm * Tsb;
        const rm::Quaternion Rms = Tsm.R.inv();

        // TODO: is it possible to not doing optixTrace twice?
        rm::Vector Dmean = {0.0, 0.0, 0.0};
        rm::Vector Mmean = {0.0, 0.0, 0.0};
        unsigned int Ncorr = 0;
        rm::Matrix3x3 C;
        C.setZeros();
        
        // Detecting Means
        for(unsigned int vid = 0; vid < Nvertical; vid++)
        {
            for(unsigned int hid = 0; hid < Nhorizontal; hid++)
            {
                // ids of results
                const unsigned int loc_id = mem.model->getBufferId(vid, hid);
                
                const float real_range = mem.ranges[loc_id];
                if(real_range < rangeMin || real_range > rangeMax)
                {
                    continue;
                }

                // Origin
                const rm::Vector ray_orig_s = mem.model->getOrigin(vid, hid);
                const rm::Vector ray_orig_m = Tsm * ray_orig_s;

                // Direction
                const rm::Vector ray_dir_s = mem.model->getDirection(vid, hid);
                const rm::Vector ray_dir_m = Tsm.R * ray_dir_s;

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

                const rm::Vector nint_m = {
                    __uint_as_float( p1 ),
                    __uint_as_float( p2 ),
                    __uint_as_float( p3 )
                };

                // going to sensor space
                const rm::Vector preal_s = ray_orig_s + ray_dir_s * real_range;
                const rm::Vector pint_s = ray_orig_s + ray_dir_s * range;

                rm::Vector nint_s = Rms * nint_m;
                
                // if(nint_s.dot(ray_dir_s) > 0.0 )
                // {
                //     nint_s *= -1.0;
                // }

                const float signed_plane_dist = (pint_s - preal_s).dot(nint_s);
                const rm::Vector pmesh_s = preal_s + nint_s * signed_plane_dist;
                const float dist_sqrt = (pmesh_s - preal_s).l2normSquared();

                if(dist_sqrt < dist_thresh * dist_thresh)
                {
                    const rm::Vector preal_b = Tsb * preal_s;
                    const rm::Vector pmesh_b = Tsb * pmesh_s;
                    // Online update: Covariance and means 
                    // - wrong: https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance 
                    // use the following equations instead
                    {
                        const float N_1 = static_cast<float>(Ncorr);
                        const float N = static_cast<float>(Ncorr + 1);
                        const float w1 = N_1/N;
                        const float w2 = 1.0/N;

                        const rm::Vector d_mean_old = Dmean;
                        const rm::Vector m_mean_old = Mmean;

                        const rm::Vector d_mean_new = d_mean_old * w1 + preal_b * w2; 
                        const rm::Vector m_mean_new = m_mean_old * w1 + pmesh_b * w2;

                        auto P1 = (pmesh_b - m_mean_new).multT(preal_b - d_mean_new);
                        auto P2 = (m_mean_old - m_mean_new).multT(d_mean_old - d_mean_new);

                        // write
                        Dmean = d_mean_new;
                        Mmean = m_mean_new;
                        C = C * w1 + P1 * w2 + P2 * w1;
                        Ncorr = Ncorr + 1;
                    }
                }
            }
        }

        mem.Ncorr[pid] = Ncorr;
        mem.C[pid] = C;
        mem.m1[pid] = Dmean;
        mem.m2[pid] = Mmean;
    }
}