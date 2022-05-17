#include "rmcl/math/math.h"
#include <Eigen/Dense>

using namespace rmagine;

namespace rmcl {

void correction_from_covs(
    const rmagine::MemoryView<rmagine::Vector, rmagine::RAM>& ms,
    const rmagine::MemoryView<rmagine::Vector, rmagine::RAM>& ds,
    const rmagine::MemoryView<rmagine::Matrix3x3, rmagine::RAM>& Cs,
    const rmagine::MemoryView<unsigned int, rmagine::RAM>& Ncorr,
    rmagine::MemoryView<rmagine::Transform, rmagine::RAM>& Tdelta)
{
    #pragma omp parallel for
    for(size_t pid=0; pid<ms.size(); pid++)
    {
        if(Ncorr[pid] > 0)
        {
            Matrix3x3 U, V;

            { // the only Eigen code left
                const Eigen::Matrix3f* Ceig = reinterpret_cast<const Eigen::Matrix3f*>(&Cs[pid]);
                Eigen::Matrix3f* Ueig = reinterpret_cast<Eigen::Matrix3f*>(&U);
                Eigen::Matrix3f* Veig = reinterpret_cast<Eigen::Matrix3f*>(&V);

                Eigen::JacobiSVD<Eigen::Matrix3f> svd(Ceig[0], Eigen::ComputeFullU | Eigen::ComputeFullV);
                Ueig[0] = svd.matrixU();
                Veig[0] = svd.matrixV();
            }

            Quaternion R;
            R.set(U * V.transpose());
            Tdelta[pid].R = R;
            Tdelta[pid].t = ds[pid] - R * ms[pid];
        } else {
            Tdelta[pid].setIdentity();
        }
    }
}

void correction_from_covs(
    const rmagine::MemoryView<rmagine::Vector, rmagine::RAM>& ms,
    const rmagine::MemoryView<rmagine::Vector, rmagine::RAM>& ds,
    const rmagine::MemoryView<rmagine::Matrix3x3, rmagine::RAM>& Cs,
    const rmagine::MemoryView<unsigned int, rmagine::RAM>& Ncorr,
    rmagine::MemoryView<rmagine::Quaternion, rmagine::RAM>& Rdelta,
    rmagine::MemoryView<rmagine::Vector, rmagine::RAM>& tdelta)
{
    #pragma omp parallel for
    for(size_t pid=0; pid<ms.size(); pid++)
    {
        if(Ncorr[pid] > 0)
        {
            Matrix3x3 U, V;

            { // the only Eigen code left
                const Eigen::Matrix3f* Ceig = reinterpret_cast<const Eigen::Matrix3f*>(&Cs[pid]);
                Eigen::Matrix3f* Ueig = reinterpret_cast<Eigen::Matrix3f*>(&U);
                Eigen::Matrix3f* Veig = reinterpret_cast<Eigen::Matrix3f*>(&V);

                Eigen::JacobiSVD<Eigen::Matrix3f> svd(Ceig[0], Eigen::ComputeFullU | Eigen::ComputeFullV);
                Ueig[0] = svd.matrixU();
                Veig[0] = svd.matrixV();
            }

            Quaternion R;
            R.set(U * V.transpose());
            Rdelta[pid] = R;
            tdelta[pid] = ds[pid] - R * ms[pid];
        } else {
            Rdelta[pid].setIdentity();
            tdelta[pid] = {0.0, 0.0, 0.0};
        }
    }
}

void weighted_average(
    const rmagine::MemoryView<rmagine::Vector, rmagine::RAM>& ms1,
    const rmagine::MemoryView<rmagine::Vector, rmagine::RAM>& ds1,
    const rmagine::MemoryView<rmagine::Matrix3x3, rmagine::RAM>& Cs1,
    const rmagine::MemoryView<unsigned int, rmagine::RAM>& Ncorr1,
    const rmagine::MemoryView<rmagine::Vector, rmagine::RAM>& ms2,
    const rmagine::MemoryView<rmagine::Vector, rmagine::RAM>& ds2,
    const rmagine::MemoryView<rmagine::Matrix3x3, rmagine::RAM>& Cs2,
    const rmagine::MemoryView<unsigned int, rmagine::RAM>& Ncorr2,
    rmagine::MemoryView<rmagine::Vector, rmagine::RAM>& ms,
    rmagine::MemoryView<rmagine::Vector, rmagine::RAM>& ds,
    rmagine::MemoryView<rmagine::Matrix3x3, rmagine::RAM>& Cs,
    rmagine::MemoryView<unsigned int, rmagine::RAM>& Ncorr)
{
    #pragma omp parallel for
    for(size_t pid=0; pid<ms1.size(); pid++)
    {
        const unsigned int Ncorr_ = Ncorr1[pid] + Ncorr2[pid];
        const float Ncorrf = static_cast<float>(Ncorr_);
        float w1 = static_cast<float>(Ncorr1[pid]) / Ncorrf;
        float w2 = static_cast<float>(Ncorr2[pid]) / Ncorrf;

        ms[pid] = ms1[pid] * w1 + ms2[pid] * w2;
        ds[pid] = ds1[pid] * w1 + ds2[pid] * w2;
        Cs[pid] = Cs1[pid] * w1 + Cs2[pid] * w2;
        Ncorr[pid] = Ncorr_;
    }
}

void weighted_average(
    const rmagine::MemoryView<rmagine::Vector, rmagine::RAM>& ms1,
    const rmagine::MemoryView<rmagine::Vector, rmagine::RAM>& ds1,
    const rmagine::MemoryView<rmagine::Matrix3x3, rmagine::RAM>& Cs1,
    const rmagine::MemoryView<unsigned int, rmagine::RAM>& Ncorr1,
    float w1,
    const rmagine::MemoryView<rmagine::Vector, rmagine::RAM>& ms2,
    const rmagine::MemoryView<rmagine::Vector, rmagine::RAM>& ds2,
    const rmagine::MemoryView<rmagine::Matrix3x3, rmagine::RAM>& Cs2,
    const rmagine::MemoryView<unsigned int, rmagine::RAM>& Ncorr2,
    float w2,
    rmagine::MemoryView<rmagine::Vector, rmagine::RAM>& ms,
    rmagine::MemoryView<rmagine::Vector, rmagine::RAM>& ds,
    rmagine::MemoryView<rmagine::Matrix3x3, rmagine::RAM>& Cs,
    rmagine::MemoryView<unsigned int, rmagine::RAM>& Ncorr)
{
    #pragma omp parallel for
    for(size_t pid=0; pid<ms1.size(); pid++)
    {
        const unsigned int Ncorr_ = Ncorr1[pid] + Ncorr2[pid];
        const float Ncorrf = static_cast<float>(Ncorr_);

        ms[pid] = ms1[pid] * w1 + ms2[pid] * w2;
        ds[pid] = ds1[pid] * w1 + ds2[pid] * w2;
        Cs[pid] = Cs1[pid] * w1 + Cs2[pid] * w2;
        Ncorr[pid] = Ncorr_;
    }
}

} // namespace rmcl