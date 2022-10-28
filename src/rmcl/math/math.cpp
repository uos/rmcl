#include "rmcl/math/math.h"
#include <Eigen/Dense>
#include <rmagine/util/prints.h>

using namespace rmagine;
namespace rm = rmagine;

namespace rmcl {

Correction::Correction()
:m_svd(new SVD)
{
    
}

Correction::Correction(SVDPtr svd)
:m_svd(svd)
{

}

void Correction::correction_from_covs(
    const rmagine::MemoryView<rmagine::Vector, rmagine::RAM>& ds,
    const rmagine::MemoryView<rmagine::Vector, rmagine::RAM>& ms,
    const rmagine::MemoryView<rmagine::Matrix3x3, rmagine::RAM>& Cs,
    const rmagine::MemoryView<unsigned int, rmagine::RAM>& Ncorr,
    rmagine::MemoryView<rmagine::Transform, rmagine::RAM>& Tdelta
) const
{
    #pragma omp parallel for if(ds.size() > 100)
    for(size_t pid=0; pid<ds.size(); pid++)
    {
        if(Ncorr[pid] > 0)
        {
            Matrix3x3 U, V, S;
            Vector s;

            m_svd->calcUSV(Cs[pid], U, s, V);

            S.setIdentity();
            if(U.det() * V.det() < 0)
            {
                S(2, 2) = -1;
            }

            Transform T;
            T.R.set(U * S * V.transpose());
            T.R.normalizeInplace();
            T.t = ms[pid] - T.R * ds[pid];

            Tdelta[pid] = T;
        } else {
            Tdelta[pid].setIdentity();
        }
    }
}


void Correction::correction_from_covs(
    const rmagine::MemoryView<rmagine::Vector, rmagine::RAM>& ds,
    const rmagine::MemoryView<rmagine::Vector, rmagine::RAM>& ms,
    const rmagine::MemoryView<rmagine::Matrix3x3, rmagine::RAM>& Cs,
    const rmagine::MemoryView<unsigned int, rmagine::RAM>& Ncorr,
    rmagine::MemoryView<rmagine::Quaternion, rmagine::RAM>& Rdelta,
    rmagine::MemoryView<rmagine::Vector, rmagine::RAM>& tdelta) const
{
    #pragma omp parallel for if(ds.size() > 100)
    for(size_t pid=0; pid<ds.size(); pid++)
    {
        if(Ncorr[pid] > 0)
        {
            Matrix3x3 U, V, S;

            m_svd->calcUV(Cs[pid], U, V);

            S.setIdentity();
            if(U.det() * V.det() < 0)
            {
                S(2, 2) = -1;
            }

            Quaternion R;
            R.set(U * S * V.transpose());
            R.normalizeInplace();
            Rdelta[pid] = R;
            tdelta[pid] = ms[pid] - R * ds[pid];
        } else {
            Rdelta[pid].setIdentity();
            tdelta[pid] = {0.0, 0.0, 0.0};
        }
    }
}


void Correction::correction_from_covs(
    const CorrectionPreResults<rmagine::RAM>& pre_res,
    rmagine::MemoryView<rmagine::Transform, rmagine::RAM>& Tdelta) const
{
    correction_from_covs(pre_res.ds, pre_res.ms, pre_res.Cs, pre_res.Ncorr, Tdelta);
}

rmagine::Memory<rmagine::Transform, rmagine::RAM> Correction::correction_from_covs(
    const CorrectionPreResults<rmagine::RAM>& pre_res) const
{
    rmagine::Memory<rmagine::Transform, rmagine::RAM> Tdelta(pre_res.ds.size());
    correction_from_covs(pre_res, Tdelta);
    return Tdelta;
}

void weighted_average(
    const rmagine::MemoryView<rmagine::Vector, rmagine::RAM>& ds1,
    const rmagine::MemoryView<rmagine::Vector, rmagine::RAM>& ms1,
    const rmagine::MemoryView<rmagine::Matrix3x3, rmagine::RAM>& Cs1,
    const rmagine::MemoryView<unsigned int, rmagine::RAM>& Ncorr1,
    const rmagine::MemoryView<rmagine::Vector, rmagine::RAM>& ds2,
    const rmagine::MemoryView<rmagine::Vector, rmagine::RAM>& ms2,
    const rmagine::MemoryView<rmagine::Matrix3x3, rmagine::RAM>& Cs2,
    const rmagine::MemoryView<unsigned int, rmagine::RAM>& Ncorr2,
    rmagine::MemoryView<rmagine::Vector, rmagine::RAM>& ds,
    rmagine::MemoryView<rmagine::Vector, rmagine::RAM>& ms,
    rmagine::MemoryView<rmagine::Matrix3x3, rmagine::RAM>& Cs,
    rmagine::MemoryView<unsigned int, rmagine::RAM>& Ncorr)
{
    #pragma omp parallel for if(ds1.size() > 100)
    for(size_t pid=0; pid<ds1.size(); pid++)
    {
        const unsigned int Ncorr_ = Ncorr1[pid] + Ncorr2[pid];
        const float Ncorrf = static_cast<float>(Ncorr_);
        float w1 = static_cast<float>(Ncorr1[pid]) / Ncorrf;
        float w2 = static_cast<float>(Ncorr2[pid]) / Ncorrf;

        ds[pid] = ds1[pid] * w1 + ds2[pid] * w2;
        ms[pid] = ms1[pid] * w1 + ms2[pid] * w2;
        Cs[pid] = Cs1[pid] * w1 + Cs2[pid] * w2;
        Ncorr[pid] = Ncorr_;
    }
}

void weighted_average(
    const rmagine::MemoryView<rmagine::Vector, rmagine::RAM>& ds1,
    const rmagine::MemoryView<rmagine::Vector, rmagine::RAM>& ms1,
    const rmagine::MemoryView<rmagine::Matrix3x3, rmagine::RAM>& Cs1,
    const rmagine::MemoryView<unsigned int, rmagine::RAM>& Ncorr1,
    float w1,
    const rmagine::MemoryView<rmagine::Vector, rmagine::RAM>& ds2,
    const rmagine::MemoryView<rmagine::Vector, rmagine::RAM>& ms2,
    const rmagine::MemoryView<rmagine::Matrix3x3, rmagine::RAM>& Cs2,
    const rmagine::MemoryView<unsigned int, rmagine::RAM>& Ncorr2,
    float w2,
    rmagine::MemoryView<rmagine::Vector, rmagine::RAM>& ds,
    rmagine::MemoryView<rmagine::Vector, rmagine::RAM>& ms,
    rmagine::MemoryView<rmagine::Matrix3x3, rmagine::RAM>& Cs,
    rmagine::MemoryView<unsigned int, rmagine::RAM>& Ncorr)
{
    // TODO: make define for 100
    #pragma omp parallel for if(ds1.size() > 100)
    for(size_t pid=0; pid<ds1.size(); pid++)
    {
        const unsigned int Ncorr_ = Ncorr1[pid] + Ncorr2[pid];
        const float Ncorrf = static_cast<float>(Ncorr_);

        const rm::Vector dsi = ds1[pid] * w1 + ds2[pid] * w2;
        const rm::Vector msi = ms1[pid] * w1 + ms2[pid] * w2;
        
        // std::cout << "weighted_average 1 - NEW IMPL" << std::endl;
        auto P1 = Cs1[pid] * w1 + Cs2[pid] * w2;
        auto P2 = (ms1[pid] - msi).multT(ds1[pid] - dsi) * w1 + (ms2[pid] - msi).multT(ds2[pid] - dsi) * w2;

        ds[pid] = dsi;
        ms[pid] = msi;
        Cs[pid] = P1 + P2;
        Ncorr[pid] = Ncorr_;
    }
}

void weighted_average(
    const std::vector<rmagine::MemoryView<rmagine::Vector, rmagine::RAM> >& dataset_means,
    const std::vector<rmagine::MemoryView<rmagine::Vector, rmagine::RAM> >& model_means,
    const std::vector<rmagine::MemoryView<rmagine::Matrix3x3, rmagine::RAM> >& covs,
    const std::vector<rmagine::MemoryView<unsigned int, rmagine::RAM> >& Ncorrs,
    rmagine::MemoryView<rmagine::Vector, rmagine::RAM>& ds,
    rmagine::MemoryView<rmagine::Vector, rmagine::RAM>& ms,
    rmagine::MemoryView<rmagine::Matrix3x3, rmagine::RAM>& Cs,
    rmagine::MemoryView<unsigned int, rmagine::RAM>& Ncorr)
{
    // std::cout << "weighted_average 2 - NEW IMPL" << std::endl;

    #pragma omp parallel for
    for(size_t pid=0; pid<ds.size(); pid++)
    {
        unsigned int Ncorr_ = 0;
        for(size_t i=0; i<dataset_means.size(); i++)
        {
            Ncorr_ += Ncorrs[i][pid];
        }

        std::vector<float> weights(dataset_means.size());
        const float Ncorrf = static_cast<float>(Ncorr_);

        for(size_t i=0; i<dataset_means.size(); i++)
        {
            weights[i] = static_cast<float>(Ncorrs[i][pid]) / Ncorrf;
        }

        Vector ms_ = {0.0, 0.0, 0.0};
        Vector ds_ = {0.0, 0.0, 0.0};
        Matrix3x3 C_;
        C_.setZeros();

        for(size_t i=0; i<dataset_means.size(); i++)
        {
            ds_ += dataset_means[i][pid] * weights[i];
            ms_ += model_means[i][pid] * weights[i];
            C_ += covs[i][pid] * weights[i];
        }

        ms[pid] = ms_;
        ds[pid] = ds_;
        Cs[pid] = C_;
        Ncorr[pid] = Ncorr_;
    }
}

void weighted_average(
    const std::vector<rmagine::MemoryView<rmagine::Vector, rmagine::RAM> >& dataset_means,
    const std::vector<rmagine::MemoryView<rmagine::Vector, rmagine::RAM> >& model_means,
    const std::vector<rmagine::MemoryView<rmagine::Matrix3x3, rmagine::RAM> >& covs,
    const std::vector<rmagine::MemoryView<unsigned int, rmagine::RAM> >& Ncorrs,
    const std::vector<float>& weights,
    rmagine::MemoryView<rmagine::Vector, rmagine::RAM>& ds,
    rmagine::MemoryView<rmagine::Vector, rmagine::RAM>& ms,
    rmagine::MemoryView<rmagine::Matrix3x3, rmagine::RAM>& Cs,
    rmagine::MemoryView<unsigned int, rmagine::RAM>& Ncorr)
{
    // std::cout << "weighted_average 3 - NEW IMPL" << std::endl;

    #pragma omp parallel for if(ds.size() > 100)
    for(size_t pid=0; pid<ds.size(); pid++)
    {
        Vector ms_ = {0.0, 0.0, 0.0};
        Vector ds_ = {0.0, 0.0, 0.0};
        Matrix3x3 C_;
        C_.setZeros();
        unsigned int Ncorr_ = 0;
        float w_ = 0.0;

        for(size_t i=0; i<dataset_means.size(); i++)
        {
            const float w =  weights[i];
            const rm::Vector Di = dataset_means[i][pid];
            const rm::Vector Mi = model_means[i][pid];
            const rm::Matrix3x3 Ci = covs[i][pid];
            
            const float w_tot = w_ + w;
            const float w1 = w_ / w_tot;
            const float w2 = w  / w_tot;

            const rm::Vector ds_old = ds_;
            const rm::Vector ms_old = ms_;

            // ds_ = w_ * ds_old + w * 
            ds_ = ds_old * w1 + Di * w2;
            ms_ = ms_old * w1 + Mi * w2;

            auto P1 = C_ * w1 + Ci * w2;
            auto P2 = (ms_old - ms_).multT(ds_old - ds_) * w1 + (Mi - ms_).multT(Di - ds_) * w2;

            C_ = P1 + P2;
            Ncorr_ += Ncorrs[i][pid];
            w_ += w;
        }

        ds[pid] = ds_;
        ms[pid] = ms_;
        Cs[pid] = C_;
        Ncorr[pid] = Ncorr_;
    }
}


void weighted_average(
    const std::vector<CorrectionPreResults<rmagine::RAM> >& pre_results,
    CorrectionPreResults<rmagine::RAM>& pre_results_combined)
{
    // std::cout << "weighted_average 4 - NEW IMPL" << std::endl;

    // source: to fuse
    std::vector<MemoryView<Vector, RAM> > ds;
    std::vector<MemoryView<Vector, RAM> > ms;
    std::vector<MemoryView<Matrix3x3, RAM> > Cs;
    std::vector<MemoryView<unsigned int, RAM> > Ncorrs;

    for(size_t i = 0; i < pre_results.size(); i++)
    {
        ds.push_back(pre_results[i].ds);
        ms.push_back(pre_results[i].ms);
        Cs.push_back(pre_results[i].Cs);
        Ncorrs.push_back(pre_results[i].Ncorr);
    }

    weighted_average(ds, ms, Cs, Ncorrs, 
        pre_results_combined.ds, pre_results_combined.ms, pre_results_combined.Cs, pre_results_combined.Ncorr);
}

CorrectionPreResults<rmagine::RAM> weighted_average(
    const std::vector<CorrectionPreResults<rmagine::RAM> >& pre_results)
{
    CorrectionPreResults<rmagine::RAM> res;
    if(pre_results.size() > 0)
    {   
        size_t Nposes = pre_results[0].Cs.size();

        res.ds.resize(Nposes);
        res.ms.resize(Nposes);
        res.Cs.resize(Nposes);
        res.Ncorr.resize(Nposes);

        weighted_average(pre_results, res);
    }

    return res;
}

void weighted_average(
    const std::vector<CorrectionPreResults<rmagine::RAM> >& pre_results,
    const std::vector<float>& weights,
    CorrectionPreResults<rmagine::RAM>& pre_results_combined)
{
    // std::cout << "weighted_average 5 - NEW IMPL" << std::endl;
    // source: to fuse
    std::vector<MemoryView<Vector, RAM> > ds;
    std::vector<MemoryView<Vector, RAM> > ms;
    std::vector<MemoryView<Matrix3x3, RAM> > Cs;
    std::vector<MemoryView<unsigned int, RAM> > Ncorrs;

    for(size_t i = 0; i < pre_results.size(); i++)
    {
        ds.push_back(pre_results[i].ds);
        ms.push_back(pre_results[i].ms);
        Cs.push_back(pre_results[i].Cs);
        Ncorrs.push_back(pre_results[i].Ncorr);
    }

    weighted_average(ds, ms, Cs, Ncorrs, weights,
        pre_results_combined.ds, pre_results_combined.ms, pre_results_combined.Cs, pre_results_combined.Ncorr);
}

CorrectionPreResults<rmagine::RAM> weighted_average(
    const std::vector<CorrectionPreResults<rmagine::RAM> >& pre_results,
    const std::vector<float>& weights)
{
    // std::cout << "weighted_average 6 - NEW IMPL" << std::endl;

    CorrectionPreResults<rmagine::RAM> res;
    if(pre_results.size() > 0)
    {
        size_t Nposes = pre_results[0].Cs.size();

        res.ds.resize(Nposes);
        res.ms.resize(Nposes);
        res.Cs.resize(Nposes);
        res.Ncorr.resize(Nposes);

        weighted_average(pre_results, weights, res);
    }
    

    return res;
}

} // namespace rmcl