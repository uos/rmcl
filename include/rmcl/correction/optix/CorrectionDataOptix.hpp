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
 * @brief Data passed to the OptiX kernels
 *
 * @date 03.10.2022
 * @author Alexander Mock
 * 
 * @copyright Copyright (c) 2022, University Osnabrück. All rights reserved.
 * This project is released under the 3-Clause BSD License.
 * 
 */

#ifndef RMCL_OPTIX_CORRECTION_DATA_HPP
#define RMCL_OPTIX_CORRECTION_DATA_HPP

#include <optix.h>
#include <cuda_runtime.h>

#include <rmagine/types/sensor_models.h>
#include <rmagine/math/types.h>
#include <rmagine/types/MemoryCuda.hpp>
#include <rmagine/simulation/optix/sim_program_data.h>


#include "rmcl/correction/CorrectionParams.hpp"



namespace rmcl {

/**
 * @brief Data for scanwise correction
 */
template<typename ModelT>
struct CorrectionDataSW
{
    // inputs
    const ModelT*                   model;
    const float*                    ranges;
    const rmagine::Transform*       Tsb; // Sensor to Base Transform
    const rmagine::Transform*       Tbm; // Base to Map transforms
    unsigned int                    Nposes;
    const CorrectionParams*         params;
    bool                            optical;
    // handle
    unsigned long long          handle;
    // outputs
    rmagine::Vector*                m1; // from, data means
    rmagine::Vector*                m2; // to, model means
    rmagine::Matrix3x3*             C; // C between 1 and 2
    unsigned int*                   Ncorr;
};

using SphereCorrectionDataSW = CorrectionDataSW<rmagine::SphericalModel>;
using PinholeCorrectionDataSW = CorrectionDataSW<rmagine::PinholeModel>;
using O1DnCorrectionDataSW = CorrectionDataSW<rmagine::O1DnModel_<rmagine::VRAM_CUDA> >;
using OnDnCorrectionDataSW = CorrectionDataSW<rmagine::OnDnModel_<rmagine::VRAM_CUDA> >;

/**
 * @brief Data for raywise correction
 * 
 */
template<typename ModelT>
struct CorrectionDataRW
{
    // inputs
    const ModelT*                   model;
    const float*                    ranges;
    const rmagine::Transform*       Tsb; // Sensor to Base Transform
    const rmagine::Transform*       Tbm; // Base to Map transforms
    unsigned int                    Nposes;
    const CorrectionParams*         params;
    bool                            optical;
    // handle
    unsigned long long              handle;
    // outputs
    rmagine::Vector*                dataset_points;
    rmagine::Vector*                model_points; // nearest points on mesh
    unsigned int*                   corr_valid;
};

using SphereCorrectionDataRW = CorrectionDataRW<rmagine::SphericalModel>;
using PinholeCorrectionDataRW = CorrectionDataRW<rmagine::PinholeModel>;
using O1DnCorrectionDataRW = CorrectionDataRW<rmagine::O1DnModel_<rmagine::VRAM_CUDA> >;
using OnDnCorrectionDataRW = CorrectionDataRW<rmagine::OnDnModel_<rmagine::VRAM_CUDA> >;

} // namespace rmcl

#endif // RMCL_OPTIX_CORRECTION_DATA_HPP