/*
 * Copyright (c) 2022, University Osnabrück
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University Osnabrück nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL University Osnabrück BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file
 * 
 * @brief Math functions for CPU Memory
 *
 * @date 03.10.2022
 * @author Alexander Mock
 * 
 * @copyright Copyright (c) 2022, University Osnabrück. All rights reserved.
 * This project is released under the 3-Clause BSD License.
 * 
 */

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
        const rmagine::MemoryView<rmagine::Vector, rmagine::RAM>& ds,
        const rmagine::MemoryView<rmagine::Vector, rmagine::RAM>& ms,
        const rmagine::MemoryView<rmagine::Matrix3x3, rmagine::RAM>& Cs,
        const rmagine::MemoryView<unsigned int, rmagine::RAM>& Ncorr,
        rmagine::MemoryView<rmagine::Transform, rmagine::RAM>& Tdelta
    ) const;

    void correction_from_covs(
        const rmagine::MemoryView<rmagine::Vector, rmagine::RAM>& ds,
        const rmagine::MemoryView<rmagine::Vector, rmagine::RAM>& ms,
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

    // TODO: put this to GPU
    inline void operator()(
        const rmagine::MemoryView<rmagine::Vector, rmagine::RAM>& ds,
        const rmagine::MemoryView<rmagine::Vector, rmagine::RAM>& ms,
        const rmagine::MemoryView<rmagine::Matrix3x3, rmagine::RAM>& Cs,
        const rmagine::MemoryView<unsigned int, rmagine::RAM>& Ncorr,
        rmagine::MemoryView<rmagine::Transform, rmagine::RAM>& Tdelta) const
    {
        correction_from_covs(ds, ms, Cs, Ncorr, Tdelta);
    }

    inline void operator()(
        const rmagine::MemoryView<rmagine::Vector, rmagine::RAM>& ds,
        const rmagine::MemoryView<rmagine::Vector, rmagine::RAM>& ms,
        const rmagine::MemoryView<rmagine::Matrix3x3, rmagine::RAM>& Cs,
        const rmagine::MemoryView<unsigned int, rmagine::RAM>& Ncorr,
        rmagine::MemoryView<rmagine::Quaternion, rmagine::RAM>& Rdelta,
        rmagine::MemoryView<rmagine::Vector, rmagine::RAM>& tdelta) const
    {
        correction_from_covs(ds, ms, Cs, Ncorr, Rdelta, tdelta);
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
    rmagine::MemoryView<unsigned int, rmagine::RAM>& Ncorr
);

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
    rmagine::MemoryView<unsigned int, rmagine::RAM>& Ncorr
);

void weighted_average(
    const std::vector<rmagine::MemoryView<rmagine::Vector, rmagine::RAM> >& dataset_means,
    const std::vector<rmagine::MemoryView<rmagine::Vector, rmagine::RAM> >& model_means,
    const std::vector<rmagine::MemoryView<rmagine::Matrix3x3, rmagine::RAM> >& covs,
    const std::vector<rmagine::MemoryView<unsigned int, rmagine::RAM> >& Ncorrs,
    rmagine::MemoryView<rmagine::Vector, rmagine::RAM>& ds,
    rmagine::MemoryView<rmagine::Vector, rmagine::RAM>& ms,
    rmagine::MemoryView<rmagine::Matrix3x3, rmagine::RAM>& Cs,
    rmagine::MemoryView<unsigned int, rmagine::RAM>& Ncorr
);

void weighted_average(
    const std::vector<rmagine::MemoryView<rmagine::Vector, rmagine::RAM> >& dataset_means,
    const std::vector<rmagine::MemoryView<rmagine::Vector, rmagine::RAM> >& model_means,
    const std::vector<rmagine::MemoryView<rmagine::Matrix3x3, rmagine::RAM> >& covs,
    const std::vector<rmagine::MemoryView<unsigned int, rmagine::RAM> >& Ncorrs,
    const std::vector<float>& weights,
    rmagine::MemoryView<rmagine::Vector, rmagine::RAM>& ds,
    rmagine::MemoryView<rmagine::Vector, rmagine::RAM>& ms,
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