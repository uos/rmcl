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
 * @brief O1DnCorrectorEmbree
 *
 * @date 03.10.2022
 * @author Alexander Mock
 * 
 * @copyright Copyright (c) 2022, University Osnabrück. All rights reserved.
 * This project is released under the 3-Clause BSD License.
 * 
 */

#ifndef RMCL_CORRECTOR_O1DN_EMBREE_HPP
#define RMCL_CORRECTOR_O1DN_EMBREE_HPP

#include <memory>

// rmagine deps
#include <rmagine/map/EmbreeMap.hpp>
#include <rmagine/types/sensor_models.h>
#include <rmagine/math/SVD.hpp>
#include <rmagine/simulation/O1DnSimulatorEmbree.hpp>

#include "CorrectionResults.hpp"
#include "CorrectionParams.hpp"

namespace rmcl {

/**
 * @brief EmbreeCorrector computes robot pose corrections in robot frame on CPU.
 * 
 * Required information to set:
 * - Sensor Model: O1DnModel
 * - Sensor Data: Scanner Ranges
 * - Transformation: Sensor to Base
 * 
 * TODO: inherit from rmagine::O1DnSimulatorEmbree ???
 */
class O1DnCorrectorEmbree 
: public rmagine::O1DnSimulatorEmbree
{
public:
    /**
     * @brief Correct Sensor data towards a given map
     * 
     * @param mesh 
     */
    using Base = rmagine::O1DnSimulatorEmbree;
    using Base::Base;

    void setParams(
        const CorrectionParams& params);

    void setInputData(
        const rmagine::MemoryView<float, rmagine::RAM>& ranges);

    /**
     * @brief Correct one ore multiple Poses towards the map
     * 
     * @param Tbm Poses represented as transformations (rmagine::Transform)
     * @return Memory<Transform, RAM> Correction in robots base coordinates
     */
    CorrectionResults<rmagine::RAM> correct(
        const rmagine::MemoryView<rmagine::Transform, rmagine::RAM>& Tbms
    );

    void computeCovs(
        const rmagine::MemoryView<rmagine::Transform, rmagine::RAM>& Tbms,
        rmagine::MemoryView<rmagine::Vector, rmagine::RAM>& data_means,
        rmagine::MemoryView<rmagine::Vector, rmagine::RAM>& model_means,
        rmagine::MemoryView<rmagine::Matrix3x3, rmagine::RAM>& Cs,
        rmagine::MemoryView<unsigned int, rmagine::RAM>& Ncorr
    );

    void computeCovs(
        const rmagine::MemoryView<rmagine::Transform, rmagine::RAM>& Tbms,
        CorrectionPreResults<rmagine::RAM>& res
    );

    CorrectionPreResults<rmagine::RAM> computeCovs(
        const rmagine::MemoryView<rmagine::Transform, rmagine::RAM>& Tbms
    );

    /**
     * @brief Find Simulative Projective Correspondences (SPC)
     * 
     * @param Tbms 
     * @param dataset_points 
     * @param model_points 
     * @param corr_valid 
     */
    void findSPC(
        const rmagine::MemoryView<rmagine::Transform, rmagine::RAM>& Tbms,
        rmagine::MemoryView<rmagine::Point> data_points,
        rmagine::MemoryView<rmagine::Point> model_points,
        rmagine::MemoryView<unsigned int> corr_valid
    );

    void findSPC(
        const rmagine::MemoryView<rmagine::Transform, rmagine::RAM>& Tbms,
        rmagine::Memory<rmagine::Point>& dataset_points,
        rmagine::Memory<rmagine::Point>& model_points,
        rmagine::Memory<unsigned int>& corr_valid
    );
    
    /**
     * @brief Find Raycasting Correspondences (RCC)
     * 
     * @param Tbms 
     * @param dataset_points 
     * @param model_points 
     * @param corr_valid
     */
    void findRCC(
        const rmagine::Transform& Tbm,
        rmagine::MemoryView<rmagine::Point> dataset_points,
        rmagine::MemoryView<rmagine::Point> model_points,
        rmagine::MemoryView<rmagine::Vector> model_normals,
        rmagine::MemoryView<unsigned int> corr_valid
    ) const;

    void findRCC(
        const rmagine::MemoryView<rmagine::Transform, rmagine::RAM>& Tbms,
        rmagine::MemoryView<rmagine::Point> data_points,
        rmagine::MemoryView<rmagine::Point> model_points,
        rmagine::MemoryView<rmagine::Vector> model_normals,
        rmagine::MemoryView<unsigned int> corr_valid
    ) const;

    void findRCC(
        const rmagine::MemoryView<rmagine::Transform, rmagine::RAM>& Tbms,
        rmagine::Memory<rmagine::Point>& dataset_points,
        rmagine::Memory<rmagine::Point>& model_points,
        rmagine::Memory<rmagine::Vector>& model_normals,
        rmagine::Memory<unsigned int>& corr_valid
    ) const;
    
    inline CorrectionParams params() const
    {
        return m_params;
    }

    // TODO: add properly - rmagine
    inline rmagine::O1DnModel model() const
    {
        return m_model[0];
    }

protected:
    rmagine::Memory<float, rmagine::RAM> m_ranges;

    CorrectionParams m_params;

    // TODO: currently unused
    rmagine::SVDPtr m_svd;
};

using O1DnCorrectorEmbreePtr = std::shared_ptr<O1DnCorrectorEmbree>;

} // namespace rmcl

#endif // RMCL_CORRECTOR_ONDN_EMBREE_HPP