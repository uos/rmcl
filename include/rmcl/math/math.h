#ifndef RMCL_MATH_MATH_H
#define RMCL_MATH_MATH_H

#include <rmagine/math/types.h>
#include <rmagine/types/Memory.hpp>
#include <vector>
#include <rmcl/correction/CorrectionResults.hpp>
#include <rmagine/math/SVD.hpp>
#include <memory>

namespace rmcl
{

class Correction {
public:

    Correction();
    Correction(rmagine::SVDPtr svd);

    void correction_from_covs(
        const rmagine::MemoryView<rmagine::Vector, rmagine::RAM>& ms,
        const rmagine::MemoryView<rmagine::Vector, rmagine::RAM>& ds,
        const rmagine::MemoryView<rmagine::Matrix3x3, rmagine::RAM>& Cs,
        const rmagine::MemoryView<unsigned int, rmagine::RAM>& Ncorr,
        rmagine::MemoryView<rmagine::Transform, rmagine::RAM>& Tdelta
    ) const;

    void correction_from_covs(
        const rmagine::MemoryView<rmagine::Vector, rmagine::RAM>& ms,
        const rmagine::MemoryView<rmagine::Vector, rmagine::RAM>& ds,
        const rmagine::MemoryView<rmagine::Matrix3x3, rmagine::RAM>& Cs,
        const rmagine::MemoryView<unsigned int, rmagine::RAM>& Ncorr,
        rmagine::MemoryView<rmagine::Quaternion, rmagine::RAM>& Rdelta,
        rmagine::MemoryView<rmagine::Vector, rmagine::RAM>& tdelta
    ) const;

    void correction_from_covs(
        const CorrectionPreResults<rmagine::RAM>& pre_res,
        rmagine::MemoryView<rmagine::Transform, rmagine::RAM>& Tdelta
    ) const;

    rmagine::Memory<rmagine::Transform, rmagine::RAM> correction_from_covs(
        const CorrectionPreResults<rmagine::RAM>& pre_res
    ) const;

    inline void operator()(
        const rmagine::MemoryView<rmagine::Vector, rmagine::RAM>& ms,
        const rmagine::MemoryView<rmagine::Vector, rmagine::RAM>& ds,
        const rmagine::MemoryView<rmagine::Matrix3x3, rmagine::RAM>& Cs,
        const rmagine::MemoryView<unsigned int, rmagine::RAM>& Ncorr,
        rmagine::MemoryView<rmagine::Transform, rmagine::RAM>& Tdelta) const
    {
        correction_from_covs(ms, ds, Cs, Ncorr, Tdelta);
    }

    inline void operator()(
        const rmagine::MemoryView<rmagine::Vector, rmagine::RAM>& ms,
        const rmagine::MemoryView<rmagine::Vector, rmagine::RAM>& ds,
        const rmagine::MemoryView<rmagine::Matrix3x3, rmagine::RAM>& Cs,
        const rmagine::MemoryView<unsigned int, rmagine::RAM>& Ncorr,
        rmagine::MemoryView<rmagine::Quaternion, rmagine::RAM>& Rdelta,
        rmagine::MemoryView<rmagine::Vector, rmagine::RAM>& tdelta) const
    {
        correction_from_covs(ms, ds, Cs, Ncorr, Rdelta, tdelta);
    }

    inline void operator()(
        const CorrectionPreResults<rmagine::RAM>& pre_res,
        rmagine::MemoryView<rmagine::Transform, rmagine::RAM>& Tdelta
    ) const
    {
        correction_from_covs(pre_res, Tdelta);
    }

    inline rmagine::Memory<rmagine::Transform, rmagine::RAM> operator()(
        const CorrectionPreResults<rmagine::RAM>& pre_res
    ) const
    {
        return correction_from_covs(pre_res);
    }

private:
    rmagine::SVDPtr m_svd;
};

using CorrectionPtr = std::shared_ptr<Correction>;

// weighted average by
// - number of correspondences
// - fixed weights
// TODO: more than two

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
    rmagine::MemoryView<unsigned int, rmagine::RAM>& Ncorr
);

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
    rmagine::MemoryView<unsigned int, rmagine::RAM>& Ncorr
);

void weighted_average(
    const std::vector<rmagine::MemoryView<rmagine::Vector, rmagine::RAM> >& model_means,
    const std::vector<rmagine::MemoryView<rmagine::Vector, rmagine::RAM> >& dataset_means,
    const std::vector<rmagine::MemoryView<rmagine::Matrix3x3, rmagine::RAM> >& covs,
    const std::vector<rmagine::MemoryView<unsigned int, rmagine::RAM> >& Ncorrs,
    rmagine::MemoryView<rmagine::Vector, rmagine::RAM>& ms,
    rmagine::MemoryView<rmagine::Vector, rmagine::RAM>& ds,
    rmagine::MemoryView<rmagine::Matrix3x3, rmagine::RAM>& Cs,
    rmagine::MemoryView<unsigned int, rmagine::RAM>& Ncorr
);

void weighted_average(
    const std::vector<rmagine::MemoryView<rmagine::Vector, rmagine::RAM> >& model_means,
    const std::vector<rmagine::MemoryView<rmagine::Vector, rmagine::RAM> >& dataset_means,
    const std::vector<rmagine::MemoryView<rmagine::Matrix3x3, rmagine::RAM> >& covs,
    const std::vector<rmagine::MemoryView<unsigned int, rmagine::RAM> >& Ncorrs,
    const std::vector<float>& weights,
    rmagine::MemoryView<rmagine::Vector, rmagine::RAM>& ms,
    rmagine::MemoryView<rmagine::Vector, rmagine::RAM>& ds,
    rmagine::MemoryView<rmagine::Matrix3x3, rmagine::RAM>& Cs,
    rmagine::MemoryView<unsigned int, rmagine::RAM>& Ncorr
);

void weighted_average(
    const std::vector<CorrectionPreResults<rmagine::RAM> >& pre_results,
    CorrectionPreResults<rmagine::RAM>& pre_results_combined
);

CorrectionPreResults<rmagine::RAM> weighted_average(
    const std::vector<CorrectionPreResults<rmagine::RAM> >& pre_results
);

void weighted_average(
    const std::vector<CorrectionPreResults<rmagine::RAM> >& pre_results,
    const std::vector<float>& weights,
    CorrectionPreResults<rmagine::RAM>& pre_results_combined
);

CorrectionPreResults<rmagine::RAM> weighted_average(
    const std::vector<CorrectionPreResults<rmagine::RAM> >& pre_results,
    const std::vector<float>& weights
);


} // namespace rmcl

#endif // RMCL_MATH_MATH_H