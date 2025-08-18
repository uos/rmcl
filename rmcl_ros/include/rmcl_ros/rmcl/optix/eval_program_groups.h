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
 * @brief OptiX Program Groups creation
 *
 * @date 03.10.2022
 * @author Alexander Mock
 * 
 * @copyright Copyright (c) 2022, University Osnabrück. All rights reserved.
 * This project is released under the 3-Clause BSD License.
 * 
 */

#ifndef RMCL_MCL_OPTIX_EVAL_PGS_H
#define RMCL_MCL_OPTIX_EVAL_PGS_H

// rmagine optix module interface
#include <rmagine/util/optix/optix_modules.h>
#include <rmagine/util/optix/OptixData.hpp>
#include <rmagine/util/optix/OptixSbtRecord.hpp>

// map connection
#include <rmagine/map/optix/OptixSceneEventReceiver.hpp>
#include <rmagine/map/optix/optix_sbt.h>

// measurement connection
#include "EvaluationDataOptix.hpp"


namespace rmcl
{

struct EvalRayGenProgramGroup 
: public rmagine::ProgramGroup
{
    using RecordData        = rmagine::RayGenDataEmpty;
    using SbtRecordData     = rmagine::SbtRecord<RecordData>;

    SbtRecordData*        record_h      = nullptr;

    virtual ~EvalRayGenProgramGroup();
};

using EvalRayGenProgramGroupPtr = std::shared_ptr<EvalRayGenProgramGroup>;


struct EvalMissProgramGroup
: public rmagine::ProgramGroup
{
    using RecordData          = rmagine::MissDataEmpty;
    using SbtRecordData       = rmagine::SbtRecord<RecordData>;

    SbtRecordData*        record_h      = nullptr;

    virtual ~EvalMissProgramGroup();
};

using EvalMissProgramGroupPtr = std::shared_ptr<EvalMissProgramGroup>;



struct EvalHitProgramGroup
: public rmagine::ProgramGroup
, public rmagine::OptixSceneEventReceiver
{
    using RecordData        = rmagine::OptixSceneSBT;
    using SbtRecordData     = rmagine::SbtRecord<RecordData>;

    SbtRecordData*        record_h      = nullptr;

    virtual void onSBTUpdated(bool size_changed) override;

    virtual ~EvalHitProgramGroup();
};

using EvalHitProgramGroupPtr = std::shared_ptr<EvalHitProgramGroup>;


// GEN
EvalRayGenProgramGroupPtr make_program_group_eval_gen(
    rmagine::OptixScenePtr scene,
    rmagine::ProgramModulePtr module);

EvalRayGenProgramGroupPtr make_program_group_eval_gen(
    rmagine::OptixScenePtr scene);

// MISS SW
EvalMissProgramGroupPtr make_program_group_eval_miss(
    rmagine::OptixScenePtr scene,
    rmagine::ProgramModulePtr module);

EvalMissProgramGroupPtr make_program_group_eval_miss(
    rmagine::OptixScenePtr scene);

// - Hit SW
EvalHitProgramGroupPtr make_program_group_eval_hit(
    rmagine::OptixScenePtr scene,
    rmagine::ProgramModulePtr module);

EvalHitProgramGroupPtr make_program_group_eval_hit(
    rmagine::OptixScenePtr scene);


} // namespace rmcl


#endif // RMCL_MCL_OPTIX_EVAL_PGS_H