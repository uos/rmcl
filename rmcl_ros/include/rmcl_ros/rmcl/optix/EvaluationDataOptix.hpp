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
#ifndef RMCL_OPTIX_EVALUATION_DATA_HPP
#define RMCL_OPTIX_EVALUATION_DATA_HPP

#include <optix.h>
#include <cuda_runtime.h>

#include <rmagine/types/sensor_models.h>
#include <rmagine/math/types.h>
#include <rmagine/types/MemoryCuda.hpp>
#include <rmagine/simulation/optix/sim_program_data.h>
#include <rmagine/types/sensor_models.h>

#include <rmcl_ros/rmcl/RangeMeasurement.hpp>
#include <rmcl_ros/rmcl/ParticleAttributes.hpp>

// #include "rmcl/rmcl/EvaluationParams.hpp"

namespace rmcl {

/**
 * @brief Data for particle-wise single measurement evaluation
 */
struct BeamEvaluationData
{
    // inputs
    RangeMeasurement                meas; // actual range measurement (in sensor coordinate system)
    rmagine::Interval               sensor_range;
    rmagine::Transform              Tsb; // sensor to base transform
    const rmagine::Transform*       particle_poses; // Base to Map transforms, i.e. particles
    ParticleAttributes*             particle_attrs; // [in/out] update likelihood
    unsigned int                    Nparticles;
    // params

    float                           dist_sigma; // dist sigma used when both sim and real meas hit
    float                           real_hit_sim_miss_error; // in meter
    float                           real_miss_sim_hit_error; // in meter
    float                           real_miss_sim_miss_error; // in meter

    // handle
    unsigned long long              handle;
    // outputs
};


} // namespace rmcl

#endif // RMCL_OPTIX_EVALUATION_DATA_HPP