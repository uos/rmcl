#include <rmcl/correction/PinholeCorrectorEmbree.hpp>
#include <Eigen/Dense>

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
    const rmagine::Memory<float, rmagine::RAM>& ranges)
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
    const rmagine::Memory<rmagine::Transform, rmagine::RAM>& Tbms)
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

        std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > D, M;
        unsigned int Ncorr = 0;

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
                    nint_m.normalize();
                    
                    // transform normal from global to local
                    nint_b = Tmb.R * nint_m;
                    // distance of real point to plane at simulated point
                    float signed_plane_dist = (preal_b - pint_b).dot(nint_b);
                    // project point to plane results in correspondence
                    const Vector pmesh_b = preal_b + nint_b * signed_plane_dist;  

                    const float distance = (pmesh_b - preal_b).l2norm();

                    if(distance < max_distance)
                    {
                        D.push_back({preal_b.x, preal_b.y, preal_b.z});
                        M.push_back({pmesh_b.x, pmesh_b.y, pmesh_b.z});
                        Ncorr++;
                    }
                }
            }
        }

        res.Ncorr[pid] = Ncorr;

        if(Ncorr > 0)
        {
            Eigen::Matrix<float, 3, -1> Dm(3, D.size());
            Eigen::Matrix<float, 3, -1> Mm(3, M.size());
            for(size_t i=0; i<D.size(); i++)
            {
                Dm(0, i) = D[i](0);
                Dm(1, i) = D[i](1);
                Dm(2, i) = D[i](2);
            }

            for(size_t i=0; i<M.size(); i++)
            {
                Mm(0, i) = M[i](0);
                Mm(1, i) = M[i](1);
                Mm(2, i) = M[i](2);
            }
            // Model and Dataset PCL are in base space.

            Eigen::Matrix4f Tm = my_umeyama(Dm, Mm);
            Eigen::Matrix3f R = Tm.block<3,3>(0,0);
            Eigen::Vector3f t = Tm.block<3,1>(0,3);

            Matrix3x3* R_rm = reinterpret_cast<Matrix3x3*>(&R);

            // std::cout << "R: " << *R_rm << std::endl;
            // std::cout << "t: " << t.transpose() << std::endl;

            res.Tdelta[pid].R.set(*R_rm);
            res.Tdelta[pid].t = {t.x(), t.y(), t.z()};
        } else {
            res.Tdelta[pid].setIdentity();
        }
    }

    return res;
}


} // namespace rmcl