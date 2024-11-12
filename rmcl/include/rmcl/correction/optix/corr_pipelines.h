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
 * @brief Optix Pipeline creation
 *
 * @date 03.10.2022
 * @author Alexander Mock
 * 
 * @copyright Copyright (c) 2022, University Osnabrück. All rights reserved.
 * This project is released under the 3-Clause BSD License.
 * 
 */

#ifndef RMCL_CORRECTION_OPTIX_CORR_PIPELINES_H
#define RMCL_CORRECTION_OPTIX_CORR_PIPELINES_H

// rmagine optix module interface
#include <rmagine/util/optix/optix_modules.h>

// map connection
#include <rmagine/map/optix/OptixSceneEventReceiver.hpp>

// sensor connection
#include "CorrectionDataOptix.hpp"


namespace rmcl
{



struct CorrPipeline 
: public rmagine::Pipeline
, public rmagine::OptixSceneEventReceiver
{
    virtual void onDepthChanged() override;

    virtual void onCommitDone(const rmagine::OptixSceneCommitResult& info) override;

    virtual ~CorrPipeline();
};

using CorrPipelinePtr = std::shared_ptr<CorrPipeline>;

// Pipeline
CorrPipelinePtr make_pipeline_corr_sw(
    rmagine::OptixScenePtr scene,
    unsigned int sensor_id);


CorrPipelinePtr make_pipeline_corr_rw(
    rmagine::OptixScenePtr scene,
    unsigned int sensor_id);

} // namespace rmcl

#endif // RMCL_CORRECTION_OPTIX_CORR_PIPELINES_H