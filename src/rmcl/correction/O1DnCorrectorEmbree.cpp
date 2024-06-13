#include <rmcl/correction/O1DnCorrectorEmbree.hpp>
#include <Eigen/Dense>

#include <rmcl/math/math.h>

#include <rmagine/math/omp.h>

// DEBUG
#include <rmagine/util/prints.h>

#include <limits>

using namespace rmagine;
namespace rm = rmagine;

namespace rmcl
{

void O1DnCorrectorEmbree::setParams(
    const CorrectionParams& params)
{
    m_params = params;
}

void O1DnCorrectorEmbree::setInputData(
    const rmagine::MemoryView<float, rmagine::RAM>& ranges)
{
    m_ranges = ranges;
}

CorrectionResults<rm::RAM> O1DnCorrectorEmbree::correct(
    const rm::MemoryView<rm::Transform, rm::RAM>& Tbms)
{
    CorrectionResults<RAM> res;
    res.Tdelta.resize(Tbms.size());
    res.Ncorr.resize(Tbms.size());

    const float max_distance = m_params.max_distance;

    auto scene = m_map->scene->handle();

    const rm::Transform Tsb = m_Tsb[0];

    #pragma omp parallel for default(shared) if(Tbms.size() > 4)
    for(size_t pid=0; pid < Tbms.size(); pid++)
    {
        const rm::Transform Tbm = Tbms[pid];
        const rm::Transform Tsm = Tbm * Tsb;
        const rm::Transform Tms = ~Tsm;

        const unsigned int glob_shift = pid * m_model->size();

        Vector Dmean = {0.0, 0.0, 0.0};
        Vector Mmean = {0.0, 0.0, 0.0};
        unsigned int Ncorr = 0;
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

                const Vector ray_orig_s = m_model->getOrigin(vid, hid);
                const Vector ray_orig_m = Tsm * ray_orig_s;

                const Vector ray_dir_s = m_model->getDirection(vid, hid);
                const Vector ray_dir_m = Tsm.R * ray_dir_s;

                RTCRayHit rayhit;
                rayhit.ray.org_x = ray_orig_m.x;
                rayhit.ray.org_y = ray_orig_m.y;
                rayhit.ray.org_z = ray_orig_m.z;
                rayhit.ray.dir_x = ray_dir_m.x;
                rayhit.ray.dir_y = ray_dir_m.y;
                rayhit.ray.dir_z = ray_dir_m.z;
                rayhit.ray.tnear = 0;
                rayhit.ray.tfar = std::numeric_limits<float>::infinity();
                rayhit.ray.mask = -1;
                rayhit.ray.flags = 0;
                rayhit.hit.geomID = RTC_INVALID_GEOMETRY_ID;
                rayhit.hit.instID[0] = RTC_INVALID_GEOMETRY_ID;

                rtcIntersect1(scene, &rayhit);
                
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
                    preal_s = ray_orig_s + ray_dir_s * range_real;

                    // search point on surface that is more nearby
                    pint_s = ray_orig_s + ray_dir_s * rayhit.ray.tfar;
                    
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
        }

        res.Ncorr[pid] = Ncorr;

        if(Ncorr > 0)
        {
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

void O1DnCorrectorEmbree::computeCovs(
    const rmagine::MemoryView<rmagine::Transform, rmagine::RAM>& Tbms,
    rmagine::MemoryView<rmagine::Vector, rmagine::RAM>& ds,
    rmagine::MemoryView<rmagine::Vector, rmagine::RAM>& ms,
    rmagine::MemoryView<rmagine::Matrix3x3, rmagine::RAM>& Cs,
    rmagine::MemoryView<unsigned int, rmagine::RAM>& Ncorr)
{
    const float max_distance = m_params.max_distance;

    auto scene = m_map->scene->handle();

    const rm::Transform Tsb = m_Tsb[0];


    #pragma omp parallel for default(shared) if(Tbms.size() > 4)
    for(size_t pid=0; pid < Tbms.size(); pid++)
    {
        const rm::Transform Tbm = Tbms[pid];
        const rm::Transform Tsm = Tbm * Tsb;
        const rm::Transform Tms = ~Tsm;

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

                const Vector ray_orig_s = m_model->getOrigin(vid, hid);
                const Vector ray_orig_m = Tsm * ray_orig_s;

                const Vector ray_dir_s = m_model->getDirection(vid, hid);
                const Vector ray_dir_m = Tsm.R * ray_dir_s;

                RTCRayHit rayhit;
                rayhit.ray.org_x = ray_orig_m.x;
                rayhit.ray.org_y = ray_orig_m.y;
                rayhit.ray.org_z = ray_orig_m.z;
                rayhit.ray.dir_x = ray_dir_m.x;
                rayhit.ray.dir_y = ray_dir_m.y;
                rayhit.ray.dir_z = ray_dir_m.z;
                rayhit.ray.tnear = 0;
                rayhit.ray.tfar = std::numeric_limits<float>::infinity();
                rayhit.ray.mask = -1;
                rayhit.ray.flags = 0;
                rayhit.hit.geomID = RTC_INVALID_GEOMETRY_ID;
                rayhit.hit.instID[0] = RTC_INVALID_GEOMETRY_ID;


                rtcIntersect1(scene, &rayhit);
                

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
                    preal_s = ray_orig_s + ray_dir_s * range_real;

                    // search point on surface that is more nearby
                    pint_s = ray_orig_s + ray_dir_s * rayhit.ray.tfar;
                    
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
                        // - wrong: https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance
                        // use the following equations instead
                        {
                            const float N_1 = static_cast<float>(Ncorr_);
                            const float N = static_cast<float>(Ncorr_ + 1);
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
                            Ncorr_ = Ncorr_ + 1;
                        }
                    }
                }
            }
        }

        Ncorr[pid] = Ncorr_;
        ds[pid] = Dmean;
        ms[pid] = Mmean;
        Cs[pid] = C;
    }
}

void O1DnCorrectorEmbree::computeCovs(
    const rmagine::MemoryView<rmagine::Transform, rmagine::RAM>& Tbms,
    CorrectionPreResults<rmagine::RAM>& res)
{
    computeCovs(Tbms, res.ds, res.ms, res.Cs, res.Ncorr);
}

CorrectionPreResults<rmagine::RAM> O1DnCorrectorEmbree::computeCovs(
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

void O1DnCorrectorEmbree::findSPC(
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
                    dataset_points[glob_id] = {0.0f, 0.0f, 0.0f};
                    model_points[glob_id] = {0.0f, 0.0f, 0.0f};
                    corr_valid[glob_id] = 0;
                    continue;
                }
                const Vector ray_orig_s = m_model->getOrigin(vid, hid);
                const Vector ray_dir_s = m_model->getDirection(vid, hid);

                const Vector ray_orig_m = Tsm * ray_orig_s;
                const Vector ray_dir_m = Tsm.R * ray_dir_s;

                RTCRayHit rayhit;
                rayhit.ray.org_x = ray_orig_m.x;
                rayhit.ray.org_y = ray_orig_m.y;
                rayhit.ray.org_z = ray_orig_m.z;
                rayhit.ray.dir_x = ray_dir_m.x;
                rayhit.ray.dir_y = ray_dir_m.y;
                rayhit.ray.dir_z = ray_dir_m.z;
                rayhit.ray.tnear = 0;
                rayhit.ray.tfar = std::numeric_limits<float>::infinity();
                rayhit.ray.mask = -1;
                rayhit.ray.flags = 0;
                rayhit.hit.geomID = RTC_INVALID_GEOMETRY_ID;
                rayhit.hit.instID[0] = RTC_INVALID_GEOMETRY_ID;

                rtcIntersect1(scene, &rayhit);

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
                    preal_s = ray_orig_s + ray_dir_s * range_real;

                    // search point on surface that is more nearby
                    pint_s = ray_orig_s + ray_dir_s * rayhit.ray.tfar;
                    
                    // transform normal from global to local
                    nint_s = Tms.R * nint_m;

                    // flip to base: no
                    // if(ray_dir_s.dot(nint_s) > 0.0)
                    // {
                    //     nint_s = -nint_s;
                    // }

                    // distance of real point to plane at simulated point
                    float signed_plane_dist = (pint_s - preal_s).dot(nint_s);
                    // project point to plane results in correspondence
                    const Vector pmesh_s = preal_s + nint_s * signed_plane_dist;  

                    const float distance = (pmesh_s - preal_s).l2norm();

                    // convert back to base (sensor shared coordinate system)
                    const Vector preal_b = Tsb * preal_s;
                    const Vector pmesh_b = Tsb * pmesh_s;

                    dataset_points[glob_id] = preal_b;
                    model_points[glob_id] = pmesh_b;

                    if(distance < max_distance)
                    {   
                        corr_valid[glob_id] = 1;
                    } else {
                        corr_valid[glob_id] = 0;
                    }
                } else {
                    dataset_points[glob_id] = {0.0f, 0.0f, 0.0f};
                    model_points[glob_id] = {0.0f, 0.0f, 0.0f};
                    corr_valid[glob_id] = 0;
                }
            }
        }
    }
}

void O1DnCorrectorEmbree::findSPC(
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


void O1DnCorrectorEmbree::findRCC(
    const rmagine::Transform& Tbm,
    rmagine::MemoryView<rmagine::Point> dataset_points,
    rmagine::MemoryView<rmagine::Point> model_points,
    rmagine::MemoryView<rmagine::Vector> model_normals,
    rmagine::MemoryView<unsigned int> corr_valid,
    rmagine::MemoryView<unsigned int> scene_ids,
    rmagine::MemoryView<unsigned int> geometry_ids) const
{
    const float max_distance = m_params.max_distance;

    auto scene = m_map->scene->handle();

    const rm::Transform Tsb = m_Tsb[0];

    const rmagine::Transform Tsm = Tbm * Tsb;
    const rmagine::Transform Tmb = ~Tbm;

    for(unsigned int vid = 0; vid < m_model->getHeight(); vid++)
    {
        for(unsigned int hid = 0; hid < m_model->getWidth(); hid++)
        {
            const unsigned int loc_id = m_model->getBufferId(vid, hid);
            const float range_real = m_ranges[loc_id];
            
            if(range_real < m_model->range.min 
                || range_real > m_model->range.max)
            {
                dataset_points[loc_id] = {0.0f, 0.0f, 0.0f};
                model_points[loc_id] = {0.0f, 0.0f, 0.0f};
                corr_valid[loc_id] = 0;
                // TODO this does not make sense.
                //scene_ids[loc_id] = -1;
                //geometry_ids[loc_id] = -1;
                continue;
            }

            const rm::Vector ray_orig_s = m_model->getOrigin(vid, hid);
            const rm::Vector ray_dir_s = m_model->getDirection(vid, hid);

            const rm::Vector ray_orig_m = Tsm * ray_orig_s;
            const rm::Vector ray_dir_m = Tsm.R * ray_dir_s;

            RTCRayHit rayhit;
            rayhit.ray.org_x = ray_orig_m.x;
            rayhit.ray.org_y = ray_orig_m.y;
            rayhit.ray.org_z = ray_orig_m.z;
            rayhit.ray.dir_x = ray_dir_m.x;
            rayhit.ray.dir_y = ray_dir_m.y;
            rayhit.ray.dir_z = ray_dir_m.z;
            rayhit.ray.tnear = 0;
            rayhit.ray.tfar = std::numeric_limits<float>::infinity();
            rayhit.ray.mask = -1;
            rayhit.ray.flags = 0;
            rayhit.hit.geomID = RTC_INVALID_GEOMETRY_ID;
            rayhit.hit.instID[0] = RTC_INVALID_GEOMETRY_ID;

            rtcIntersect1(scene, &rayhit);

            bool sim_valid = rayhit.hit.geomID != RTC_INVALID_GEOMETRY_ID;
            if(sim_valid)
            {
                // map space
                rm::Vector nint_m;
                nint_m.x = rayhit.hit.Ng_x;
                nint_m.y = rayhit.hit.Ng_y;
                nint_m.z = rayhit.hit.Ng_z;
                nint_m.normalizeInplace();

                // Do point to plane ICP here
                rm::Vector preal_s, pint_s, nint_s;
                preal_s = ray_orig_s + ray_dir_s * range_real;

                // search point on surface that is more nearby
                pint_s = ray_orig_s + ray_dir_s * rayhit.ray.tfar;


                // convert back to base (sensor shared coordinate system)
                const rm::Vector preal_b = Tsb * preal_s;
                const rm::Vector pint_b = Tsb * pint_s;
                const rm::Vector nint_b = Tmb.R * nint_m;

                dataset_points[loc_id] = preal_b;
                model_points[loc_id] = pint_b;
                model_normals[loc_id] = nint_b;
                corr_valid[loc_id] = 1;
                geometry_ids[loc_id] = rayhit.hit.geomID;
                scene_ids[loc_id] = rayhit.hit.instID[0];
//                if(rayhit.hit.instID[0] > 1)
//                {
//                  std::cout << rayhit.hit.geomID << " " << rayhit.hit.instID[0] <<  std::endl;
//
//                }

            } else {
                dataset_points[loc_id] = {0.0f, 0.0f, 0.0f};
                model_points[loc_id] = {0.0f, 0.0f, 0.0f};
                model_normals[loc_id] = {0.0f, 0.0f, 0.0f};
                corr_valid[loc_id] = 0;
            }
        }
    }
}

void O1DnCorrectorEmbree::findRCC(
    const rm::MemoryView<rm::Transform, rm::RAM>& Tbms,
    rm::MemoryView<rm::Point> dataset_points,
    rm::MemoryView<rm::Point> model_points,
    rm::MemoryView<rm::Vector> model_normals,
    rm::MemoryView<unsigned int> corr_valid,
    rmagine::MemoryView<unsigned int> scene_ids,
    rmagine::MemoryView<unsigned int> geometry_ids) const

{
    const float max_distance = m_params.max_distance;

    auto scene = m_map->scene->handle();

    const rm::Transform Tsb = m_Tsb[0];

    #pragma omp parallel for default(shared) if(Tbms.size() > 4)
    for(size_t pid=0; pid < Tbms.size(); pid++)
    {
        const unsigned int glob_shift = pid * m_model->size();
        auto dataset_points_ = dataset_points(glob_shift, glob_shift + m_model->size());
        auto model_points_ = model_points(glob_shift, glob_shift + m_model->size());
        auto model_normals_ = model_normals(glob_shift, glob_shift + m_model->size());
        auto corr_valid_ = corr_valid(glob_shift, glob_shift + m_model->size());
        auto scene_ids_ = scene_ids(glob_shift, glob_shift + m_model->size());
        auto geometry_ids_ = geometry_ids(glob_shift, glob_shift + m_model->size());
        findRCC(Tbms[pid], dataset_points_, model_points_, model_normals_, corr_valid_, scene_ids_, geometry_ids_);
    }
}

void O1DnCorrectorEmbree::findRCC(
    const rmagine::MemoryView<rmagine::Transform, rmagine::RAM>& Tbms,
    rmagine::Memory<rmagine::Point>& dataset_points,
    rmagine::Memory<rmagine::Point>& model_points,
    rmagine::Memory<rmagine::Vector>& model_normals,
    rmagine::Memory<unsigned int>& corr_valid,
    rmagine::Memory<unsigned int>& scene_ids,
    rmagine::Memory<unsigned int>& geometry_ids) const
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

    if(model_normals.size() < Nrays)
    {
        model_normals.resize(Nrays);
    }

    if(corr_valid.size() < Nrays)
    {
        corr_valid.resize(Nrays);
    }

    if(scene_ids.size() < Nrays)
    {
        scene_ids.resize(Nrays);
    }

    if(geometry_ids.size() < Nrays)
    {
        geometry_ids.resize(Nrays);
    }


    findRCC(Tbms, 
        dataset_points(0, Nrays), 
        model_points(0, Nrays), 
        model_normals(0, Nrays), 
        corr_valid(0, Nrays),
        scene_ids(0, Nrays),
        geometry_ids(0, Nrays)
        );
}

void O1DnCorrectorEmbree::findCPC(
    const rmagine::Transform& Tbm,
    rmagine::MemoryView<rmagine::Point> dataset_points,
    rmagine::MemoryView<rmagine::Point> model_points,
    rmagine::MemoryView<rmagine::Vector> model_normals,
    rmagine::MemoryView<unsigned int> corr_valid) const
{
    // TODO: check if max distance should be used here.
    // - contra point: how would we know here which actual distance is used for optimization?: point to plane for example
    // - pro point: we could still set the max_distance value to inf to produce the same behavior
    const float max_distance = m_params.max_distance;

    auto scene = m_map->scene->handle();

    const rm::Transform Tsb = m_Tsb[0];

    const rmagine::Transform Tsm = Tbm * Tsb;
    const rmagine::Transform Tmb = ~Tbm;

    for(unsigned int vid = 0; vid < m_model->getHeight(); vid++)
    {
        for(unsigned int hid = 0; hid < m_model->getWidth(); hid++)
        {
            const unsigned int loc_id = m_model->getBufferId(vid, hid);
            const float range_real = m_ranges[loc_id];
            
            if(range_real < m_model->range.min 
                || range_real > m_model->range.max)
            {
                dataset_points[loc_id] = {0.0f, 0.0f, 0.0f};
                model_points[loc_id] = {0.0f, 0.0f, 0.0f};
                corr_valid[loc_id] = 0;
                continue;
            }

            const rm::Vector ray_orig_s = m_model->getOrigin(vid, hid);
            const rm::Vector ray_dir_s = m_model->getDirection(vid, hid);

            const rm::Vector ray_orig_m = Tsm * ray_orig_s;
            const rm::Vector ray_dir_m = Tsm.R * ray_dir_s;

            const rm::Point P_est_m = ray_orig_m + ray_dir_m * range_real;

            // use embree's closest point functionality (TODO: improve speed)
            const rm::EmbreeClosestPointResult res = m_map->closestPoint(P_est_m, max_distance);

            const bool res_valid = res.geomID != RTC_INVALID_GEOMETRY_ID && res.primID != RTC_INVALID_GEOMETRY_ID;

            if(res_valid)
            {
                // map space
                rm::Vector nint_m = res.n;
                nint_m.normalizeInplace();
                rm::Vector nint_b = Tmb.R * nint_m;

                const rm::Point pint_m = res.p;
                const rm::Point pint_b = Tmb * pint_m;

                const rm::Vector preal_s = ray_orig_s + ray_dir_s * range_real;
                const rm::Vector preal_b = Tsb * preal_s;
                
                dataset_points[loc_id] = preal_b;
                model_points[loc_id] = pint_b;
                model_normals[loc_id] = nint_b;
                corr_valid[loc_id] = 1;


            } else {
                dataset_points[loc_id] = {0.0f, 0.0f, 0.0f};
                model_points[loc_id] = {0.0f, 0.0f, 0.0f};
                model_normals[loc_id] = {0.0f, 0.0f, 0.0f};
                corr_valid[loc_id] = 0;
            }
        }
    }
}

void O1DnCorrectorEmbree::findCPC(
    const rm::MemoryView<rm::Transform, rm::RAM>& Tbms,
    rm::MemoryView<rm::Point> dataset_points,
    rm::MemoryView<rm::Point> model_points,
    rm::MemoryView<rm::Vector> model_normals,
    rm::MemoryView<unsigned int> corr_valid) const
{
    // const float max_distance = m_params.max_distance;

    // auto scene = m_map->scene->handle();

    // const rm::Transform Tsb = m_Tsb[0];

    #pragma omp parallel for default(shared) if(Tbms.size() > 4)
    for(size_t pid=0; pid < Tbms.size(); pid++)
    {
        const unsigned int glob_shift = pid * m_model->size();
        auto dataset_points_ = dataset_points(glob_shift, glob_shift + m_model->size());
        auto model_points_ = model_points(glob_shift, glob_shift + m_model->size());
        auto model_normals_ = model_normals(glob_shift, glob_shift + m_model->size());
        auto corr_valid_ = corr_valid(glob_shift, glob_shift + m_model->size());
        findCPC(Tbms[pid], dataset_points_, model_points_, model_normals_, corr_valid_);
    }
}

void O1DnCorrectorEmbree::findCPC(
    const rmagine::MemoryView<rmagine::Transform, rmagine::RAM>& Tbms,
    rmagine::Memory<rmagine::Point>& dataset_points,
    rmagine::Memory<rmagine::Point>& model_points,
    rmagine::Memory<rmagine::Vector>& model_normals,
    rmagine::Memory<unsigned int>& corr_valid) const
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

    if(model_normals.size() < Nrays)
    {
        model_normals.resize(Nrays);
    }

    if(corr_valid.size() < Nrays)
    {
        corr_valid.resize(Nrays);
    }

    findCPC(Tbms, 
        dataset_points(0, Nrays),
        model_points(0, Nrays),
        model_normals(0, Nrays),
        corr_valid(0, Nrays));
}

} // namespace rmcl
