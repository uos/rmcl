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

void correction_from_covs(
    const CorrectionPreResults<rmagine::RAM>& pre_res,
    rmagine::MemoryView<rmagine::Transform, rmagine::RAM>& Tdelta)
{
    correction_from_covs(pre_res.ms, pre_res.ds, pre_res.Cs, pre_res.Ncorr, Tdelta);
}

rmagine::Memory<rmagine::Transform, rmagine::RAM> correction_from_covs(
    const CorrectionPreResults<rmagine::RAM>& pre_res)
{
    rmagine::Memory<rmagine::Transform, rmagine::RAM> Tdelta(pre_res.ms.size());
    correction_from_covs(pre_res, Tdelta);
    return Tdelta;
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

void weighted_average(
    const std::vector<rmagine::MemoryView<rmagine::Vector, rmagine::RAM> >& model_means,
    const std::vector<rmagine::MemoryView<rmagine::Vector, rmagine::RAM> >& dataset_means,
    const std::vector<rmagine::MemoryView<rmagine::Matrix3x3, rmagine::RAM> >& covs,
    const std::vector<rmagine::MemoryView<unsigned int, rmagine::RAM> >& Ncorrs,
    rmagine::MemoryView<rmagine::Vector, rmagine::RAM>& ms,
    rmagine::MemoryView<rmagine::Vector, rmagine::RAM>& ds,
    rmagine::MemoryView<rmagine::Matrix3x3, rmagine::RAM>& Cs,
    rmagine::MemoryView<unsigned int, rmagine::RAM>& Ncorr)
{
    #pragma omp parallel for
    for(size_t pid=0; pid<ms.size(); pid++)
    {
        unsigned int Ncorr_ = 0;
        for(size_t i=0; i<model_means.size(); i++)
        {
            Ncorr_ += Ncorrs[i][pid];
        }

        std::vector<float> weights(model_means.size());
        const float Ncorrf = static_cast<float>(Ncorr_);

        for(size_t i=0; i<model_means.size(); i++)
        {
            weights[i] = static_cast<float>(Ncorrs[i][pid]) / Ncorrf;
        }

        Vector ms_ = {0.0, 0.0, 0.0};
        Vector ds_ = {0.0, 0.0, 0.0};
        Matrix3x3 C_;
        C_.setZeros();

        for(size_t i=0; i<model_means.size(); i++)
        {
            ms_ += model_means[i][pid] * weights[i];
            ds_ += dataset_means[i][pid] * weights[i];
            C_ += covs[i][pid] * weights[i];
        }

        ms[pid] = ms_;
        ds[pid] = ds_;
        Cs[pid] = C_;
        Ncorr[pid] = Ncorr_;
    }
}

void weighted_average(
    const std::vector<rmagine::MemoryView<rmagine::Vector, rmagine::RAM> >& model_means,
    const std::vector<rmagine::MemoryView<rmagine::Vector, rmagine::RAM> >& dataset_means,
    const std::vector<rmagine::MemoryView<rmagine::Matrix3x3, rmagine::RAM> >& covs,
    const std::vector<rmagine::MemoryView<unsigned int, rmagine::RAM> >& Ncorrs,
    const std::vector<float>& weights,
    rmagine::MemoryView<rmagine::Vector, rmagine::RAM>& ms,
    rmagine::MemoryView<rmagine::Vector, rmagine::RAM>& ds,
    rmagine::MemoryView<rmagine::Matrix3x3, rmagine::RAM>& Cs,
    rmagine::MemoryView<unsigned int, rmagine::RAM>& Ncorr)
{
    #pragma omp parallel for
    for(size_t pid=0; pid<ms.size(); pid++)
    {
        unsigned int Ncorr_ = 0;
        for(size_t i=0; i<model_means.size(); i++)
        {
            Ncorr_ += Ncorrs[i][pid];
        }

        // std::vector<float> weights(model_means.size());
        // const float Ncorrf = static_cast<float>(Ncorr_);

        // for(size_t i=0; i<model_means.size(); i++)
        // {
        //     weights[i] = static_cast<float>(Ncorrs[i][pid]) / Ncorrf;
        // }

        Vector ms_ = {0.0, 0.0, 0.0};
        Vector ds_ = {0.0, 0.0, 0.0};
        Matrix3x3 C_;
        C_.setZeros();

        for(size_t i=0; i<model_means.size(); i++)
        {
            ms_ += model_means[i][pid] * weights[i];
            ds_ += dataset_means[i][pid] * weights[i];
            C_ += covs[i][pid] * weights[i];
        }

        ms[pid] = ms_;
        ds[pid] = ds_;
        Cs[pid] = C_;
        Ncorr[pid] = Ncorr_;
    }
}


void weighted_average(
    const std::vector<CorrectionPreResults<rmagine::RAM> >& pre_results,
    CorrectionPreResults<rmagine::RAM>& pre_results_combined)
{
    // source: to fuse
    std::vector<MemoryView<Vector, RAM> > ms;
    std::vector<MemoryView<Vector, RAM> > ds;
    std::vector<MemoryView<Matrix3x3, RAM> > Cs;
    std::vector<MemoryView<unsigned int, RAM> > Ncorrs;

    for(size_t i = 0; i < pre_results.size(); i++)
    {
        ms.push_back(pre_results[i].ms);
        ds.push_back(pre_results[i].ds);
        Cs.push_back(pre_results[i].Cs);
        Ncorrs.push_back(pre_results[i].Ncorr);
    }

    weighted_average(ms, ds, Cs, Ncorrs, 
        pre_results_combined.ms, pre_results_combined.ds, pre_results_combined.Cs, pre_results_combined.Ncorr);
}

CorrectionPreResults<rmagine::RAM> weighted_average(
    const std::vector<CorrectionPreResults<rmagine::RAM> >& pre_results
)
{
    CorrectionPreResults<rmagine::RAM> res;
    size_t Nposes = pre_results[0].Cs.size();

    res.ms.resize(Nposes);
    res.ds.resize(Nposes);
    res.Cs.resize(Nposes);
    res.Ncorr.resize(Nposes);

    weighted_average(pre_results, res);

    return res;
}

void weighted_average(
    const std::vector<CorrectionPreResults<rmagine::RAM> >& pre_results,
    const std::vector<float>& weights,
    CorrectionPreResults<rmagine::RAM>& pre_results_combined)
{
    // source: to fuse
    std::vector<MemoryView<Vector, RAM> > ms;
    std::vector<MemoryView<Vector, RAM> > ds;
    std::vector<MemoryView<Matrix3x3, RAM> > Cs;
    std::vector<MemoryView<unsigned int, RAM> > Ncorrs;

    for(size_t i = 0; i < pre_results.size(); i++)
    {
        ms.push_back(pre_results[i].ms);
        ds.push_back(pre_results[i].ds);
        Cs.push_back(pre_results[i].Cs);
        Ncorrs.push_back(pre_results[i].Ncorr);
    }

    weighted_average(ms, ds, Cs, Ncorrs, weights,
        pre_results_combined.ms, pre_results_combined.ds, pre_results_combined.Cs, pre_results_combined.Ncorr);
}

CorrectionPreResults<rmagine::RAM> weighted_average(
    const std::vector<CorrectionPreResults<rmagine::RAM> >& pre_results,
    const std::vector<float>& weights)
{
    CorrectionPreResults<rmagine::RAM> res;
    size_t Nposes = pre_results[0].Cs.size();

    res.ms.resize(Nposes);
    res.ds.resize(Nposes);
    res.Cs.resize(Nposes);
    res.Ncorr.resize(Nposes);

    weighted_average(pre_results, weights, res);

    return res;
}

} // namespace rmcl