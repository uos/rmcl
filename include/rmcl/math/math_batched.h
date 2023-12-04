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
 * @brief Math functions for batched CPU memory
 *
 * @date 03.10.2022
 * @author Alexander Mock
 * 
 * @copyright Copyright (c) 2022, University Osnabrück. All rights reserved.
 * This project is released under the 3-Clause BSD License.
 * 
 */

#ifndef RMCL_MATH_BATCHED_H
#define RMCL_MATH_BATCHED_H

#include <rmagine/types/Memory.hpp>
#include <rmagine/math/types.h>

namespace rmcl
{

/**
 * @brief two-pass means and covariance computation
 * 
 * @param dataset_points 
 * @param model_points 
 * @param mask 
 * @param dataset_center 
 * @param model_center 
 * @param Cs 
 * @param Ncorr 
 */
void means_covs_batched(
    const rmagine::MemoryView<rmagine::Vector, rmagine::RAM>& dataset_points, // from
    const rmagine::MemoryView<rmagine::Vector, rmagine::RAM>& model_points, // to
    const rmagine::MemoryView<unsigned int, rmagine::RAM>& mask,
    rmagine::MemoryView<rmagine::Vector, rmagine::RAM>& dataset_center,
    rmagine::MemoryView<rmagine::Vector, rmagine::RAM>& model_center,
    rmagine::MemoryView<rmagine::Matrix3x3, rmagine::RAM>& Cs,
    rmagine::MemoryView<unsigned int, rmagine::RAM>& Ncorr);

/**
 * @brief one-pass means and covariance computation
 * 
 * @param dataset_points 
 * @param model_points 
 * @param mask 
 * @param dataset_center 
 * @param model_center 
 * @param Cs 
 * @param Ncorr 
 */
void means_covs_online_batched(
    const rmagine::MemoryView<rmagine::Vector, rmagine::RAM>& dataset_points, // from
    const rmagine::MemoryView<rmagine::Vector, rmagine::RAM>& model_points, // to
    const rmagine::MemoryView<unsigned int, rmagine::RAM>& mask,
    rmagine::MemoryView<rmagine::Vector, rmagine::RAM>& dataset_center,
    rmagine::MemoryView<rmagine::Vector, rmagine::RAM>& model_center,
    rmagine::MemoryView<rmagine::Matrix3x3, rmagine::RAM>& Cs,
    rmagine::MemoryView<unsigned int, rmagine::RAM>& Ncorr);

// Poses: N
// Scan size: M
void means_covs_p2l_online_batched(
    const rmagine::MemoryView<rmagine::Transform, rmagine::RAM>& pre_transforms, // N
    const rmagine::MemoryView<rmagine::Vector, rmagine::RAM>& dataset_points, // from, M
    const rmagine::MemoryView<unsigned int, rmagine::RAM>& dataset_mask, // from, M
    const rmagine::MemoryView<rmagine::Vector, rmagine::RAM>& model_points, // to, NxM
    const rmagine::MemoryView<rmagine::Vector, rmagine::RAM>& model_normals, // to, NxM
    const rmagine::MemoryView<unsigned int, rmagine::RAM>& model_mask, // NxM
    const float max_corr_dist,
    rmagine::MemoryView<rmagine::Vector, rmagine::RAM>& dataset_center,
    rmagine::MemoryView<rmagine::Vector, rmagine::RAM>& model_center,
    rmagine::MemoryView<rmagine::Matrix3x3, rmagine::RAM>& Cs,
    rmagine::MemoryView<unsigned int, rmagine::RAM>& Ncorr);


/**
 * @brief one-pass means and covariance computation
 * 
 * @param dataset_points 
 * @param model_points 
 * @param mask 
 * @param dataset_center 
 * @param model_center 
 * @param Cs 
 * @param Ncorr 
 */
void means_covs_online_approx_batched(
    const rmagine::MemoryView<rmagine::Vector, rmagine::RAM>& dataset_points, // from
    const rmagine::MemoryView<rmagine::Vector, rmagine::RAM>& model_points, // to
    const rmagine::MemoryView<unsigned int, rmagine::RAM>& mask,
    rmagine::MemoryView<rmagine::Vector, rmagine::RAM>& dataset_center,
    rmagine::MemoryView<rmagine::Vector, rmagine::RAM>& model_center,
    rmagine::MemoryView<rmagine::Matrix3x3, rmagine::RAM>& Cs,
    rmagine::MemoryView<unsigned int, rmagine::RAM>& Ncorr);


} // namespace rmcl

#endif // RMCL_MATH_BATCHED_CUH