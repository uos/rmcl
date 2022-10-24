#include <rmcl/correction/PinholeCorrectorEmbree.hpp>
#include <Eigen/Dense>

#include <rmagine/math/omp.h>

#include <rmagine/util/prints.h>

// DEBUG
// #include <rmagine/util/prints.h>

using namespace rmagine;

namespace rmcl
{

void PinholeCorrectorEmbree::setParams(
    const CorrectionParams& params)
{
    m_params = params;
}

void PinholeCorrectorEmbree::setInputData(
    const rmagine::MemoryView<float, rmagine::RAM>& ranges)
{
    m_ranges = ranges;
}

void PinholeCorrectorEmbree::setOptical(bool optical)
{
    m_optical = optical;
}

static Eigen::Matrix4f my_umeyama(
    const Eigen::Matrix<float, 3, -1>& from, 
    const Eigen::Matrix<float, 3, -1>& to)
{
    // (M, N): (Dim, N measurements)
    const size_t N = to.cols();

    const Eigen::Vector3f from_mean = from.rowwise().mean();
    const Eigen::Vector3f to_mean = to.rowwise().mean();

    const auto from_centered = from.colwise() - from_mean;
    const auto to_centered = to.colwise() - to_mean;

    float N_d = from.cols();
    
    // covariance matrix
    const Eigen::Matrix3f C = (from_centered * to_centered.transpose()) / N_d;

    // singular value decomposition of covariance matrix. classic ICP solving
    Eigen::JacobiSVD<Eigen::Matrix3f> svd(C, Eigen::ComputeFullU | Eigen::ComputeFullV);

    // why?
    Eigen::Vector3f S = Eigen::Vector3f::Ones(3);
    if( svd.matrixU().determinant() * svd.matrixV().determinant() < 0 )
    {
        std::cout << "my_umeyama special case occurred !!! TODO find out why" << std::endl;
        S(2) = -1;
    }

    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    // rotational part
    T.block<3,3>(0,0).noalias() = svd.matrixU() * S.asDiagonal() * svd.matrixV().transpose();
    // translational part
    T.block<3,1>(0,3).noalias() = from_mean - T.topLeftCorner(3,3) * to_mean;
    return T;
}

CorrectionResults<rmagine::RAM> PinholeCorrectorEmbree::correct(
    const rmagine::MemoryView<rmagine::Transform, rmagine::RAM>& Tbms)
{
    CorrectionResults<RAM> res;
    res.Tdelta.resize(Tbms.size());
    res.Ncorr.resize(Tbms.size());

    const float max_distance = m_params.max_distance;

    auto scene = m_map->scene->handle();

    #pragma omp parallel for
    for(size_t pid=0; pid < Tbms.size(); pid++)
    {
        const rmagine::Transform Tbm = Tbms[pid];
        const rmagine::Transform Tsb = m_Tsb[0];
        const rmagine::Transform Tsm = Tbm * Tsb;
        const rmagine::Transform Tmb = ~Tbm;

        const unsigned int glob_shift = pid * m_model->size();

        Vector Dmean = {0.0, 0.0, 0.0};
        Vector Mmean = {0.0, 0.0, 0.0};
        unsigned int Ncorr = 0;
        Matrix3x3 C;
        C.setZeros();

        // #pragma omp parallel for
        for(unsigned int vid = 0; vid < m_model->getHeight(); vid++)
        {
            Vector Dmean_inner = {0.0, 0.0, 0.0};
            Vector Mmean_inner = {0.0, 0.0, 0.0};
            unsigned int Ncorr_inner = 0;
            Matrix3x3 C_inner;
            C_inner.setZeros();

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

                Vector ray_dir_s;
                if(m_optical)
                {
                    ray_dir_s = m_model->getDirectionOptical(vid, hid);
                } else {
                    ray_dir_s = m_model->getDirection(vid, hid);
                }

                const Vector ray_dir_b = Tsb.R * ray_dir_s;
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
                    // Do point to plane ICP here
                    Vector preal_b, pint_b, nint_m, nint_b;
                    preal_b = ray_dir_b * range_real;

                    // search point on surface that is more nearby
                    pint_b = ray_dir_b * rayhit.ray.tfar;
                    nint_m.x = rayhit.hit.Ng_x;
                    nint_m.y = rayhit.hit.Ng_y;
                    nint_m.z = rayhit.hit.Ng_z;
                    nint_m.normalizeInplace();
                    
                    // transform normal from global to local
                    nint_b = Tmb.R * nint_m;
                    // distance of real point to plane at simulated point
                    float signed_plane_dist = (preal_b - pint_b).dot(nint_b);
                    // project point to plane results in correspondence
                    const Vector pmesh_b = preal_b + nint_b * signed_plane_dist;  

                    const float distance = (pmesh_b - preal_b).l2norm();

                    if(distance < max_distance)
                    {
                        Dmean_inner += preal_b;
                        Mmean_inner += pmesh_b;
                        C_inner += preal_b.multT(pmesh_b);
                        Ncorr_inner++;
                    }
                }
            }

            // reduction
            Dmean += Dmean_inner;
            Mmean += Mmean_inner;
            Ncorr += Ncorr_inner;
            C += C_inner;
        }

        res.Ncorr[pid] = Ncorr;

        if(Ncorr > 0)
        {
            Dmean /= Ncorr;
            Mmean /= Ncorr;
            C /= Ncorr;

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

            if(U.det() * V.det() < 0.0)
            {
                S(2,2) = -1.0;
            }

            res.Tdelta[pid].R.set( U * S * V.transpose() );
            res.Tdelta[pid].t = Dmean - res.Tdelta[pid].R * Mmean;
        } else {
            res.Tdelta[pid].setIdentity();
        }
    }

    return res;
}


void PinholeCorrectorEmbree::computeCovs(
    const rmagine::MemoryView<rmagine::Transform, rmagine::RAM>& Tbms,
    rmagine::MemoryView<rmagine::Vector, rmagine::RAM>& ms,
    rmagine::MemoryView<rmagine::Vector, rmagine::RAM>& ds,
    rmagine::MemoryView<rmagine::Matrix3x3, rmagine::RAM>& Cs,
    rmagine::MemoryView<unsigned int, rmagine::RAM>& Ncorr)
{
    // std::cout << "compute covs" << std::endl;
    // std::cout << Tbms[0] << std::endl;

    // std::cout << "- max dist: " << m_params.max_distance << std::endl;
    const float max_distance = m_params.max_distance;

    auto scene = m_map->scene->handle();

    // std::cout << "- rays: " << m_model->getWidth() * m_model->getHeight() << std::endl;

    #pragma omp parallel for default(shared) if(Tbms.size() > 4)
    for(size_t pid=0; pid < Tbms.size(); pid++)
    {
        const rmagine::Transform Tbm = Tbms[pid];
        const rmagine::Transform Tsb = m_Tsb[0];
        const rmagine::Transform Tsm = Tbm * Tsb;
        const rmagine::Transform Tmb = ~Tbm;

        const unsigned int glob_shift = pid * m_model->size();

        Vector Dmean = {0.0, 0.0, 0.0};
        Vector Mmean = {0.0, 0.0, 0.0};
        unsigned int Ncorr_ = 0;
        // unsigned int Ndist_ = 0;
        // unsigned int Nnohit_ = 0;
        // unsigned int Nnomeas_ = 0;

        Matrix3x3 C;
        C.setZeros();

        // #pragma omp parallel for default(shared) reduction(+:Dmean,Mmean,Ncorr_,C)
        for(unsigned int vid = 0; vid < m_model->getHeight(); vid++)
        {
            Vector Dmean_inner = {0.0, 0.0, 0.0};
            Vector Mmean_inner = {0.0, 0.0, 0.0};
            unsigned int Ncorr_inner = 0;
            Matrix3x3 C_inner;
            C_inner.setZeros();

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

                Vector ray_dir_s;
                if(m_optical)
                {
                    ray_dir_s = m_model->getDirectionOptical(vid, hid);
                } else {
                    ray_dir_s = m_model->getDirection(vid, hid);
                }

                const Vector ray_dir_b = Tsb.R * ray_dir_s;
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
                    // Do point to plane ICP here
                    Vector preal_b, pint_b, nint_m, nint_b;
                    preal_b = ray_dir_b * range_real;

                    // search point on surface that is more nearby
                    pint_b = ray_dir_b * rayhit.ray.tfar;
                    nint_m.x = rayhit.hit.Ng_x;
                    nint_m.y = rayhit.hit.Ng_y;
                    nint_m.z = rayhit.hit.Ng_z;
                    nint_m.normalizeInplace();
                    
                    // transform normal from global to local
                    nint_b = Tmb.R * nint_m;
                    // distance of real point to plane at simulated point
                    float signed_plane_dist = (preal_b - pint_b).dot(nint_b);
                    // project point to plane results in correspondence
                    const Vector pmesh_b = preal_b + nint_b * signed_plane_dist;  

                    const float distance = (pmesh_b - preal_b).l2norm();

                    if(distance < max_distance)
                    {
                        Dmean_inner += preal_b;
                        Mmean_inner += pmesh_b;
                        C_inner += preal_b.multT(pmesh_b);
                        Ncorr_inner++;
                    }
                }
            }

            // reduction
            Dmean += Dmean_inner;
            Mmean += Mmean_inner;
            Ncorr_ += Ncorr_inner;
            C += C_inner;
        }

        Ncorr[pid] = Ncorr_;

        if(Ncorr_ > 0)
        {
            Dmean /= Ncorr_;
            Mmean /= Ncorr_;
            C /= Ncorr_;
            
            ms[pid] = Mmean;
            ds[pid] = Dmean;
            Cs[pid] = C;
        } else {
            ms[pid] = {0.0, 0.0, 0.0};
            ds[pid] = {0.0, 0.0, 0.0};
            Cs[pid].setZeros();
        }
    }
}

void PinholeCorrectorEmbree::computeCovs(
    const rmagine::MemoryView<rmagine::Transform, rmagine::RAM>& Tbms,
    CorrectionPreResults<rmagine::RAM>& res)
{
    computeCovs(Tbms, res.ms, res.ds, res.Cs, res.Ncorr);
}

CorrectionPreResults<rmagine::RAM> PinholeCorrectorEmbree::computeCovs(
    const rmagine::MemoryView<rmagine::Transform, rmagine::RAM>& Tbms)
{
    CorrectionPreResults<rmagine::RAM> res;
    res.ms.resize(Tbms.size());
    res.ds.resize(Tbms.size());
    res.Cs.resize(Tbms.size());
    res.Ncorr.resize(Tbms.size());

    computeCovs(Tbms, res);

    return res;
}

} // namespace rmcl