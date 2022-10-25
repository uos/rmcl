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
 * @brief Math function for GPU memory
 *
 * @date 03.10.2022
 * @author Alexander Mock
 * 
 * @copyright Copyright (c) 2022, University Osnabrück. All rights reserved.
 * This project is released under the 3-Clause BSD License.
 * 
 */

#ifndef RMCL_MATH_MATH_CUH
#define RMCL_MATH_MATH_CUH

#include <rmagine/math/types.h>
#include <rmagine/types/MemoryCuda.hpp>
#include <vector>
#include <rmcl/correction/CorrectionResults.hpp>
#include <rmagine/math/SVDCuda.hpp>
#include <memory>

namespace rmcl {

class CorrectionCuda {
public:
    CorrectionCuda(rmagine::SVDCudaPtr svd);
    CorrectionCuda();
    CorrectionCuda(rmagine::CudaStreamPtr stream);
    CorrectionCuda(rmagine::CudaContextPtr ctx);

    void correction_from_covs(
        const rmagine::MemoryView<rmagine::Vector, rmagine::VRAM_CUDA>& ds,
        const rmagine::MemoryView<rmagine::Vector, rmagine::VRAM_CUDA>& ms,
        const rmagine::MemoryView<rmagine::Matrix3x3, rmagine::VRAM_CUDA>& Cs,
        const rmagine::MemoryView<unsigned int, rmagine::VRAM_CUDA>& Ncorr,
        rmagine::MemoryView<rmagine::Transform, rmagine::VRAM_CUDA>& Tdelta
    ) const;

    void correction_from_covs(
        const rmagine::MemoryView<rmagine::Vector, rmagine::VRAM_CUDA>& ds,
        const rmagine::MemoryView<rmagine::Vector, rmagine::VRAM_CUDA>& ms,
        const rmagine::MemoryView<rmagine::Matrix3x3, rmagine::VRAM_CUDA>& Cs,
        const rmagine::MemoryView<unsigned int, rmagine::VRAM_CUDA>& Ncorr,
        rmagine::MemoryView<rmagine::Quaternion, rmagine::VRAM_CUDA>& Rdelta,
        rmagine::MemoryView<rmagine::Vector, rmagine::VRAM_CUDA>& tdelta
    ) const;

    void correction_from_covs(
        const CorrectionPreResults<rmagine::VRAM_CUDA>& pre_res,
        rmagine::MemoryView<rmagine::Transform, rmagine::VRAM_CUDA>& Tdelta
    ) const;

    rmagine::Memory<rmagine::Transform, rmagine::VRAM_CUDA> correction_from_covs(
        const CorrectionPreResults<rmagine::VRAM_CUDA>& pre_res
    ) const;

private:
    rmagine::SVDCudaPtr m_svd;
};

using CorrectionCudaPtr = std::shared_ptr<CorrectionCuda>;

void compute_transform(
    const rmagine::MemoryView<rmagine::Matrix3x3, rmagine::VRAM_CUDA>& Us,
    const rmagine::MemoryView<rmagine::Matrix3x3, rmagine::VRAM_CUDA>& Vs,
    const rmagine::MemoryView<rmagine::Vector, rmagine::VRAM_CUDA>& ds,
    const rmagine::MemoryView<rmagine::Vector, rmagine::VRAM_CUDA>& ms,
    rmagine::MemoryView<rmagine::Transform, rmagine::VRAM_CUDA>& dT);

// weighted average by
// - number of correspondences
// - fixed weights
// TODO: more than two

void weighted_average(
    const rmagine::MemoryView<rmagine::Vector, rmagine::VRAM_CUDA>& ds1,
    const rmagine::MemoryView<rmagine::Vector, rmagine::VRAM_CUDA>& ms1,
    const rmagine::MemoryView<rmagine::Matrix3x3, rmagine::VRAM_CUDA>& Cs1,
    const rmagine::MemoryView<unsigned int, rmagine::VRAM_CUDA>& Ncorr1,
    const rmagine::MemoryView<rmagine::Vector, rmagine::VRAM_CUDA>& ds2,
    const rmagine::MemoryView<rmagine::Vector, rmagine::VRAM_CUDA>& ms2,
    const rmagine::MemoryView<rmagine::Matrix3x3, rmagine::VRAM_CUDA>& Cs2,
    const rmagine::MemoryView<unsigned int, rmagine::VRAM_CUDA>& Ncorr2,
    rmagine::MemoryView<rmagine::Vector, rmagine::VRAM_CUDA>& ds,
    rmagine::MemoryView<rmagine::Vector, rmagine::VRAM_CUDA>& ms,
    rmagine::MemoryView<rmagine::Matrix3x3, rmagine::VRAM_CUDA>& Cs,
    rmagine::MemoryView<unsigned int, rmagine::VRAM_CUDA>& Ncorr
);

void weighted_average(
    const rmagine::MemoryView<rmagine::Vector, rmagine::VRAM_CUDA>& ds1,
    const rmagine::MemoryView<rmagine::Vector, rmagine::VRAM_CUDA>& ms1,
    const rmagine::MemoryView<rmagine::Matrix3x3, rmagine::VRAM_CUDA>& Cs1,
    const rmagine::MemoryView<unsigned int, rmagine::VRAM_CUDA>& Ncorr1,
    float w1,
    const rmagine::MemoryView<rmagine::Vector, rmagine::VRAM_CUDA>& ds2,
    const rmagine::MemoryView<rmagine::Vector, rmagine::VRAM_CUDA>& ms2,
    const rmagine::MemoryView<rmagine::Matrix3x3, rmagine::VRAM_CUDA>& Cs2,
    const rmagine::MemoryView<unsigned int, rmagine::VRAM_CUDA>& Ncorr2,
    float w2,
    rmagine::MemoryView<rmagine::Vector, rmagine::VRAM_CUDA>& ds,
    rmagine::MemoryView<rmagine::Vector, rmagine::VRAM_CUDA>& ms,
    rmagine::MemoryView<rmagine::Matrix3x3, rmagine::VRAM_CUDA>& Cs,
    rmagine::MemoryView<unsigned int, rmagine::VRAM_CUDA>& Ncorr
);

void weighted_average(
    const std::vector<rmagine::MemoryView<rmagine::Vector, rmagine::VRAM_CUDA> >& dataset_means,
    const std::vector<rmagine::MemoryView<rmagine::Vector, rmagine::VRAM_CUDA> >& model_means,
    const std::vector<rmagine::MemoryView<rmagine::Matrix3x3, rmagine::VRAM_CUDA> >& covs,
    const std::vector<rmagine::MemoryView<unsigned int, rmagine::VRAM_CUDA> >& Ncorrs,
    rmagine::MemoryView<rmagine::Vector, rmagine::VRAM_CUDA>& ds,
    rmagine::MemoryView<rmagine::Vector, rmagine::VRAM_CUDA>& ms,
    rmagine::MemoryView<rmagine::Matrix3x3, rmagine::VRAM_CUDA>& Cs,
    rmagine::MemoryView<unsigned int, rmagine::VRAM_CUDA>& Ncorr
);

void weighted_average(
    const std::vector<rmagine::MemoryView<rmagine::Vector, rmagine::VRAM_CUDA> >& dataset_means,
    const std::vector<rmagine::MemoryView<rmagine::Vector, rmagine::VRAM_CUDA> >& model_means,
    const std::vector<rmagine::MemoryView<rmagine::Matrix3x3, rmagine::VRAM_CUDA> >& covs,
    const std::vector<rmagine::MemoryView<unsigned int, rmagine::VRAM_CUDA> >& Ncorrs,
    const std::vector<float>& weights,
    rmagine::MemoryView<rmagine::Vector, rmagine::VRAM_CUDA>& ds,
    rmagine::MemoryView<rmagine::Vector, rmagine::VRAM_CUDA>& ms,
    rmagine::MemoryView<rmagine::Matrix3x3, rmagine::VRAM_CUDA>& Cs,
    rmagine::MemoryView<unsigned int, rmagine::VRAM_CUDA>& Ncorr
);

void weighted_average(
    const std::vector<CorrectionPreResults<rmagine::VRAM_CUDA> >& pre_results,
    CorrectionPreResults<rmagine::VRAM_CUDA>& pre_results_combined
);

CorrectionPreResults<rmagine::VRAM_CUDA> weighted_average(
    const std::vector<CorrectionPreResults<rmagine::VRAM_CUDA> >& pre_results
);

void weighted_average(
    const std::vector<CorrectionPreResults<rmagine::VRAM_CUDA> >& pre_results,
    const std::vector<float>& weights,
    CorrectionPreResults<rmagine::VRAM_CUDA>& pre_results_combined
);

CorrectionPreResults<rmagine::VRAM_CUDA> weighted_average(
    const std::vector<CorrectionPreResults<rmagine::VRAM_CUDA> >& pre_results,
    const std::vector<float>& weights
);

} // namespace rmcl

#endif // RMCL_MATH_MATH_CUH