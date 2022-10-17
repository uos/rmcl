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
 * @brief OnDnCorrectorOptix
 *
 * @date 03.10.2022
 * @author Alexander Mock
 * 
 * @copyright Copyright (c) 2022, University Osnabrück. All rights reserved.
 * This project is released under the 3-Clause BSD License.
 * 
 */

#ifndef RMCL_ONDN_CORRECTOR_OPTIX_HPP
#define RMCL_ONDN_CORRECTOR_OPTIX_HPP

#include <rmagine/map/OptixMap.hpp>
#include <rmagine/types/sensor_models.h>
#include <rmagine/math/types.h>
#include <rmagine/math/SVDCuda.hpp>
#include <rmagine/simulation/OnDnSimulatorOptix.hpp>

#include "CorrectionResults.hpp"
#include "CorrectionParams.hpp"

#include <memory>

namespace rmcl 
{

class OnDnCorrectorOptix 
: public rmagine::OnDnSimulatorOptix
{
public:
    using Base = rmagine::OnDnSimulatorOptix;

    OnDnCorrectorOptix(rmagine::OptixMapPtr map);

    void setParams(
        const CorrectionParams& params);

    void setInputData(
        const rmagine::MemoryView<float, rmagine::RAM>& ranges
    );

    void setInputData(
        const rmagine::MemoryView<float, rmagine::VRAM_CUDA>& ranges
    );

    CorrectionResults<rmagine::VRAM_CUDA> correct(
        const rmagine::MemoryView<rmagine::Transform, rmagine::VRAM_CUDA>& Tbms
    ) const;

    void compute_covs(
        const rmagine::MemoryView<rmagine::Transform, rmagine::VRAM_CUDA>& Tbms,
        rmagine::MemoryView<rmagine::Vector, rmagine::VRAM_CUDA>& ms,
        rmagine::MemoryView<rmagine::Vector, rmagine::VRAM_CUDA>& ds,
        rmagine::MemoryView<rmagine::Matrix3x3, rmagine::VRAM_CUDA>& Cs,
        rmagine::MemoryView<unsigned int, rmagine::VRAM_CUDA>& Ncorr
    ) const;

    void compute_covs(
        const rmagine::MemoryView<rmagine::Transform, rmagine::VRAM_CUDA>& Tbms,
        CorrectionPreResults<rmagine::VRAM_CUDA>& res
    ) const;

    CorrectionPreResults<rmagine::VRAM_CUDA> compute_covs(
        const rmagine::MemoryView<rmagine::Transform, rmagine::VRAM_CUDA>& Tbms
    ) const;
    
protected:
    rmagine::Memory<float, rmagine::VRAM_CUDA> m_ranges;
    rmagine::Memory<CorrectionParams, rmagine::VRAM_CUDA> m_params;

    rmagine::SVDCudaPtr m_svd;

private:
    void computeMeansCovsRW(
        const rmagine::MemoryView<rmagine::Transform, rmagine::VRAM_CUDA>& Tbm,
        rmagine::MemoryView<rmagine::Vector, rmagine::VRAM_CUDA>& m1, // from, dataset
        rmagine::MemoryView<rmagine::Vector, rmagine::VRAM_CUDA>& m2, // to, model
        rmagine::MemoryView<rmagine::Matrix3x3, rmagine::VRAM_CUDA>& Cs,
        rmagine::MemoryView<unsigned int, rmagine::VRAM_CUDA>& Ncorr
        ) const;

    void computeMeansCovsSW(
        const rmagine::MemoryView<rmagine::Transform, rmagine::VRAM_CUDA>& Tbm,
        rmagine::MemoryView<rmagine::Vector, rmagine::VRAM_CUDA>& m1,
        rmagine::MemoryView<rmagine::Vector, rmagine::VRAM_CUDA>& m2,
        rmagine::MemoryView<rmagine::Matrix3x3, rmagine::VRAM_CUDA>& Cs,
        rmagine::MemoryView<unsigned int, rmagine::VRAM_CUDA>& Ncorr
        ) const;
};

using OnDnCorrectorOptixPtr = std::shared_ptr<OnDnCorrectorOptix>;

} // namespace rmcl

#endif // RMCL_ONDN_CORRECTOR_OPTIX_HPP