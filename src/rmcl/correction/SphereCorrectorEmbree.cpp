#include <rmcl/correction/SphereCorrectorEmbree.hpp>
#include <Eigen/Dense>

#include <rmagine/util/StopWatch.hpp>
#include <rmagine/util/prints.h>
#include <rmagine/math/omp.h>

#include <rmcl/math/math.h>


// DEBUG
// #include <rmagine/util/prints.h>
using namespace rmagine;
namespace rm = rmagine;

namespace rmcl
{

void SphereCorrectorEmbree::setParams(
    const CorrectionParams& params)
{
    m_params = params;
}

void SphereCorrectorEmbree::setInputData(
    const rmagine::MemoryView<float, rmagine::RAM>& ranges)
{
    m_ranges = ranges;
}

CorrectionResults<rmagine::RAM> SphereCorrectorEmbree::correct(
    const rmagine::MemoryView<rmagine::Transform, rmagine::RAM>& Tbms)
{
    CorrectionResults<RAM> res;
    res.Tdelta.resize(Tbms.size());
    res.Ncorr.resize(Tbms.size());

    const float max_distance = m_params.max_distance;

    auto scene = m_map->scene->handle();

    #pragma omp parallel for default(shared) if(Tbms.size() > 4)
    for(size_t pid=0; pid < Tbms.size(); pid++)
    {
        const rmagine::Transform Tbm = Tbms[pid];
        const rmagine::Transform Tsb = m_Tsb[0];
        const rmagine::Transform Tsm = Tbm * Tsb;
        const rmagine::Transform Tms = ~Tsm;

        const unsigned int glob_shift = pid * m_model->size();

        Vector Dmean = {0.0, 0.0, 0.0};
        Vector Mmean = {0.0, 0.0, 0.0};
        unsigned int Ncorr = 0;
        Matrix3x3 C;
        C.setZeros();
        
        // #pragma omp parallel for default(shared) reduction(+:Dmean, Mmean, Ncorr, C)
        for(unsigned int vid = 0; vid < m_model->getHeight(); vid++)
        {
            // #pragma omp parallel for default(shared) reduction(+:C_inner,Ncorr_inner)
            for(unsigned int hid = 0; hid < m_model->getWidth(); hid++)
            {
                const unsigned int loc_id = m_model->getBufferId(vid, hid);
                const unsigned int glob_id = glob_shift + loc_id;

                const float range_real = m_ranges[loc_id];
                if(range_real < m_model->range.min 
                    || range_real > m_model->range.max)
                {
                    continue;
                }

                const Vector ray_dir_s = m_model->getDirection(vid, hid);
                const Vector ray_dir_m = Tsm.R * ray_dir_s;

                RTCRayHit rayhit;
                rayhit.ray.org_x = Tsm.t.x;
                rayhit.ray.org_y = Tsm.t.y;
                rayhit.ray.org_z = Tsm.t.z;
                rayhit.ray.dir_x = ray_dir_m.x;
                rayhit.ray.dir_y = ray_dir_m.y;
                rayhit.ray.dir_z = ray_dir_m.z;
                rayhit.ray.tnear = 0;
                rayhit.ray.tfar = INFINITY;
                rayhit.ray.mask = 0;
                rayhit.ray.flags = 0;
                rayhit.hit.geomID = RTC_INVALID_GEOMETRY_ID;
                rayhit.hit.instID[0] = RTC_INVALID_GEOMETRY_ID;

                rtcIntersect1(scene, &m_context, &rayhit);

                bool sim_valid = rayhit.hit.geomID != RTC_INVALID_GEOMETRY_ID;
                if(sim_valid)
                {
                    Vector nint_m;
                    nint_m.x = rayhit.hit.Ng_x;
                    nint_m.y = rayhit.hit.Ng_y;
                    nint_m.z = rayhit.hit.Ng_z;
                    nint_m.normalizeInplace();

                    // Do point to plane ICP here
                    Vector preal_s, pint_s, nint_s;
                    preal_s = ray_dir_s * range_real;

                    // search point on surface that is more nearby
                    pint_s = ray_dir_s * rayhit.ray.tfar;
                    
                    
                    // transform normal from global to local
                    nint_s = Tms.R * nint_m;

                    // flip to base: check if this is really needed: no
                    // if(ray_dir_s.dot(nint_s) > 0.0)
                    // {
                    //     nint_s = -nint_s;
                    // }

                    // distance of real point to plane at simulated point
                    const float signed_plane_dist = (pint_s - preal_s).dot(nint_s);
                    // project point to plane results in correspondence
                    const Vector pmesh_s = preal_s + nint_s * signed_plane_dist;  

                    const float distance = (pmesh_s - preal_s).l2norm();

                    if(distance < max_distance)
                    {
                        const Vector preal_b = Tsb * preal_s;
                        const Vector pmesh_b = Tsb * pmesh_s;

                        // Online update: Covariance and means 
                        // - https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance
                        {
                            Ncorr++;
                            const float N = static_cast<float>(Ncorr);

                            const Vector dD = preal_b - Dmean;
                            const Vector dM = pmesh_b - Mmean;

                            // reduction
                            Dmean += dD / N;
                            Mmean += dM / N;
                            C += dM.multT(dD);
                        }
                    }
                }
            }
        }

        res.Ncorr[pid] = Ncorr;

        if(Ncorr > 0)
        {
            const float Ncorr_f = static_cast<float>(Ncorr);
            C /= Ncorr_f;

            Matrix3x3 U, V, S;

            { // the only Eigen code left
                const Eigen::Matrix3f* Ceig = reinterpret_cast<const Eigen::Matrix3f*>(&C);
                Eigen::Matrix3f* Ueig = reinterpret_cast<Eigen::Matrix3f*>(&U);
                Eigen::Matrix3f* Veig = reinterpret_cast<Eigen::Matrix3f*>(&V);

                Eigen::JacobiSVD<Eigen::Matrix3f> svd(Ceig[0], Eigen::ComputeFullU | Eigen::ComputeFullV);
                Ueig[0] = svd.matrixU();
                Veig[0] = svd.matrixV();
            }

            S.setIdentity();
            if(U.det() * V.det() < 0)
            {
                S(2, 2) = -1;
            }

            res.Tdelta[pid].R.set( U * S * V.transpose() );
            res.Tdelta[pid].t = Mmean - res.Tdelta[pid].R * Dmean;
        } else {
            res.Tdelta[pid].setIdentity();
        }
    }

    return res;
}

void SphereCorrectorEmbree::computeCovs(
    const rmagine::MemoryView<rmagine::Transform, rmagine::RAM>& Tbms,
    rmagine::MemoryView<rmagine::Vector, rmagine::RAM>& ds,
    rmagine::MemoryView<rmagine::Vector, rmagine::RAM>& ms,
    rmagine::MemoryView<rmagine::Matrix3x3, rmagine::RAM>& Cs,
    rmagine::MemoryView<unsigned int, rmagine::RAM>& Ncorr)
{
    const float max_distance = m_params.max_distance;

    auto scene = m_map->scene->handle();

    const rmagine::Transform Tsb = m_Tsb[0];

    #pragma omp parallel for default(shared) if(Tbms.size() > 4)
    for(size_t pid=0; pid < Tbms.size(); pid++)
    {
        const rmagine::Transform Tbm = Tbms[pid];
        const rmagine::Transform Tsm = Tbm * Tsb;
        const rmagine::Transform Tms = ~Tsm;

        const unsigned int glob_shift = pid * m_model->size();

        Vector Dmean = {0.0, 0.0, 0.0};
        Vector Mmean = {0.0, 0.0, 0.0};
        unsigned int Ncorr_ = 0;
        Matrix3x3 C;
        C.setZeros();

        for(unsigned int vid = 0; vid < m_model->getHeight(); vid++)
        {
            for(unsigned int hid = 0; hid < m_model->getWidth(); hid++)
            {
                const unsigned int loc_id = m_model->getBufferId(vid, hid);
                const unsigned int glob_id = glob_shift + loc_id;

                const float range_real = m_ranges[loc_id];
                
                if(range_real < m_model->range.min 
                    || range_real > m_model->range.max)
                {
                    continue;
                }
                
                const Vector ray_dir_s = m_model->getDirection(vid, hid);
                const Vector ray_dir_m = Tsm.R * ray_dir_s;

                RTCRayHit rayhit;
                rayhit.ray.org_x = Tsm.t.x;
                rayhit.ray.org_y = Tsm.t.y;
                rayhit.ray.org_z = Tsm.t.z;
                rayhit.ray.dir_x = ray_dir_m.x;
                rayhit.ray.dir_y = ray_dir_m.y;
                rayhit.ray.dir_z = ray_dir_m.z;
                rayhit.ray.tnear = 0;
                rayhit.ray.tfar = INFINITY;
                rayhit.ray.mask = 0;
                rayhit.ray.flags = 0;
                rayhit.hit.geomID = RTC_INVALID_GEOMETRY_ID;
                rayhit.hit.instID[0] = RTC_INVALID_GEOMETRY_ID;

                rtcIntersect1(scene, &m_context, &rayhit);

                bool sim_valid = rayhit.hit.geomID != RTC_INVALID_GEOMETRY_ID;
                if(sim_valid)
                {
                    // map space
                    Vector nint_m;
                    nint_m.x = rayhit.hit.Ng_x;
                    nint_m.y = rayhit.hit.Ng_y;
                    nint_m.z = rayhit.hit.Ng_z;
                    nint_m.normalizeInplace();

                    // Do point to plane ICP here
                    Vector preal_s, pint_s, nint_s;
                    preal_s = ray_dir_s * range_real;

                    // search point on surface that is more nearby
                    pint_s = ray_dir_s * rayhit.ray.tfar;
                    
                    // transform normal from global to local
                    nint_s = Tms.R * nint_m;

                    // flip to base: check if this is really needed: no
                    // if(ray_dir_s.dot(nint_s) > 0.0)
                    // {
                    //     nint_s = -nint_s;
                    // }

                    // distance of real point to plane at simulated point
                    float signed_plane_dist = (pint_s - preal_s).dot(nint_s);
                    // project point to plane results in correspondence
                    const Vector pmesh_s = preal_s + nint_s * signed_plane_dist;  

                    const float distance = (pmesh_s - preal_s).l2norm();

                    if(distance < max_distance)
                    {
                        const Vector preal_b = Tsb * preal_s;
                        const Vector pmesh_b = Tsb * pmesh_s;

                        // Online update: Covariance and means 
                        // - https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance

                        {
                            const Vector dD = preal_b - Dmean;
                            const Vector dM = pmesh_b - Mmean;
                            float N = static_cast<float>(Ncorr_ + 1);

                            // reduction
                            Ncorr_++;
                            Mmean += dM / N;
                            Dmean += dD / N;
                            C += dM.multT(dD);
                        }
                    }
                }
            }
        }

        Ncorr[pid] = Ncorr_;

        if(Ncorr_ > 0)
        {
            const float Ncorr_f = static_cast<float>(Ncorr_);
            C /= Ncorr_f;
            
            ds[pid] = Dmean;
            ms[pid] = Mmean;
            Cs[pid] = C;
        } else {
            ds[pid] = {0.0, 0.0, 0.0};
            ms[pid] = {0.0, 0.0, 0.0};
            Cs[pid].setZeros();
        }
    }
}

void SphereCorrectorEmbree::computeCovs(
    const rmagine::MemoryView<rmagine::Transform, rmagine::RAM>& Tbms,
    CorrectionPreResults<rmagine::RAM>& res)
{
    computeCovs(Tbms, res.ds, res.ms, res.Cs, res.Ncorr);
}

CorrectionPreResults<rmagine::RAM> SphereCorrectorEmbree::computeCovs(
    const rmagine::MemoryView<rmagine::Transform, rmagine::RAM>& Tbms)
{
    CorrectionPreResults<rmagine::RAM> res;
    res.ds.resize(Tbms.size());
    res.ms.resize(Tbms.size());
    res.Cs.resize(Tbms.size());
    res.Ncorr.resize(Tbms.size());

    computeCovs(Tbms, res);

    return res;
}

void SphereCorrectorEmbree::findSPC(
    const rm::MemoryView<rm::Transform, rm::RAM>& Tbms,
    rm::MemoryView<rm::Point> dataset_points,
    rm::MemoryView<rm::Point> model_points,
    rm::MemoryView<unsigned int> corr_valid)
{
    const float max_distance = m_params.max_distance;

    auto scene = m_map->scene->handle();

    const rm::Transform Tsb = m_Tsb[0];

    #pragma omp parallel for default(shared) if(Tbms.size() > 4)
    for(size_t pid=0; pid < Tbms.size(); pid++)
    {
        const rmagine::Transform Tbm = Tbms[pid];
        const rmagine::Transform Tsm = Tbm * Tsb;
        const rmagine::Transform Tms = ~Tsm;

        const unsigned int glob_shift = pid * m_model->size();

        for(unsigned int vid = 0; vid < m_model->getHeight(); vid++)
        {
            for(unsigned int hid = 0; hid < m_model->getWidth(); hid++)
            {
                const unsigned int loc_id = m_model->getBufferId(vid, hid);
                const unsigned int glob_id = glob_shift + loc_id;

                const float range_real = m_ranges[loc_id];
                
                if(range_real < m_model->range.min 
                    || range_real > m_model->range.max)
                {
                    corr_valid[glob_id] = 0;
                    continue;
                }
                
                const Vector ray_dir_s = m_model->getDirection(vid, hid);
                const Vector ray_dir_m = Tsm.R * ray_dir_s;

                RTCRayHit rayhit;
                rayhit.ray.org_x = Tsm.t.x;
                rayhit.ray.org_y = Tsm.t.y;
                rayhit.ray.org_z = Tsm.t.z;
                rayhit.ray.dir_x = ray_dir_m.x;
                rayhit.ray.dir_y = ray_dir_m.y;
                rayhit.ray.dir_z = ray_dir_m.z;
                rayhit.ray.tnear = 0;
                rayhit.ray.tfar = INFINITY;
                rayhit.ray.mask = 0;
                rayhit.ray.flags = 0;
                rayhit.hit.geomID = RTC_INVALID_GEOMETRY_ID;
                rayhit.hit.instID[0] = RTC_INVALID_GEOMETRY_ID;

                rtcIntersect1(scene, &m_context, &rayhit);

                bool sim_valid = rayhit.hit.geomID != RTC_INVALID_GEOMETRY_ID;
                if(sim_valid)
                {
                    // map space
                    Vector nint_m;
                    nint_m.x = rayhit.hit.Ng_x;
                    nint_m.y = rayhit.hit.Ng_y;
                    nint_m.z = rayhit.hit.Ng_z;
                    nint_m.normalizeInplace();

                    // Do point to plane ICP here
                    Vector preal_s, pint_s, nint_s;
                    preal_s = ray_dir_s * range_real;

                    // search point on surface that is more nearby
                    pint_s = ray_dir_s * rayhit.ray.tfar;
                    
                    // transform normal from global to local
                    nint_s = Tms.R * nint_m;

                    // flip to base: not required
                    // if(ray_dir_s.dot(nint_s) > 0.0)
                    // {
                    //     nint_s = -nint_s;
                    // }

                    // distance of real point to plane at simulated point
                    float signed_plane_dist = (pint_s - preal_s).dot(nint_s);
                    // project point to plane results in correspondence
                    const Vector pmesh_s = preal_s + nint_s * signed_plane_dist;  

                    const float distance = (pmesh_s - preal_s).l2norm();

                    if(distance < max_distance)
                    {
                        // convert back to base (sensor shared coordinate system)
                        const Vector preal_b = Tsb * preal_s;
                        const Vector pmesh_b = Tsb * pmesh_s;

                        dataset_points[glob_id] = preal_b;
                        model_points[glob_id] = pmesh_b;
                        corr_valid[glob_id] = 1;
                    } else {
                        corr_valid[glob_id] = 0;
                    }
                } else {
                    corr_valid[glob_id] = 0;
                }
            }
        }
    }
}

void SphereCorrectorEmbree::findSPC(
    const rmagine::MemoryView<rmagine::Transform, rmagine::RAM>& Tbms,
    rmagine::Memory<rmagine::Point>& dataset_points,
    rmagine::Memory<rmagine::Point>& model_points,
    rmagine::Memory<unsigned int>& corr_valid)
{
    size_t Nrays = Tbms.size() * m_model->size();
    if(dataset_points.size() < Nrays)
    {
        dataset_points.resize(Nrays);
    }

    if(model_points.size() < Nrays)
    {
        model_points.resize(Nrays);
    }

    if(corr_valid.size() < Nrays)
    {
        corr_valid.resize(Nrays);
    }

    findSPC(Tbms, dataset_points(0, Nrays), model_points(0, Nrays), corr_valid(0, Nrays));
}

} // namespace rmcl