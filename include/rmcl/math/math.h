#ifndef RMCL_MATH_MATH_H
#define RMCL_MATH_MATH_H

#include <rmagine/math/types.h>
#include <rmagine/types/Memory.hpp>
#include <vector>

namespace rmcl
{

void correction_from_covs(
    const rmagine::MemoryView<rmagine::Vector, rmagine::RAM>& ms,
    const rmagine::MemoryView<rmagine::Vector, rmagine::RAM>& ds,
    const rmagine::MemoryView<rmagine::Matrix3x3, rmagine::RAM>& Cs,
    const rmagine::MemoryView<unsigned int, rmagine::RAM>& Ncorr,
    rmagine::MemoryView<rmagine::Transform, rmagine::RAM>& Tdelta
);

void correction_from_covs(
    const rmagine::MemoryView<rmagine::Vector, rmagine::RAM>& ms,
    const rmagine::MemoryView<rmagine::Vector, rmagine::RAM>& ds,
    const rmagine::MemoryView<rmagine::Matrix3x3, rmagine::RAM>& Cs,
    const rmagine::MemoryView<unsigned int, rmagine::RAM>& Ncorr,
    rmagine::MemoryView<rmagine::Quaternion, rmagine::RAM>& Rdelta,
    rmagine::MemoryView<rmagine::Vector, rmagine::RAM>& tdelta
);

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
    const std::vector<float> weights,
    rmagine::MemoryView<rmagine::Vector, rmagine::RAM>& ms,
    rmagine::MemoryView<rmagine::Vector, rmagine::RAM>& ds,
    rmagine::MemoryView<rmagine::Matrix3x3, rmagine::RAM>& Cs,
    rmagine::MemoryView<unsigned int, rmagine::RAM>& Ncorr
);


} // namespace rmcl

#endif // RMCL_MATH_MATH_H