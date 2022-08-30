#ifndef RMCL_CORRECTION_OPTIX_CORR_PGS_H
#define RMCL_CORRECTION_OPTIX_CORR_PGS_H

// rmagine optix module interface
#include <rmagine/util/optix/optix_modules.h>
#include <rmagine/util/optix/OptixData.hpp>
#include <rmagine/util/optix/OptixSbtRecord.hpp>

// map connection
#include <rmagine/map/optix/OptixSceneEventReceiver.hpp>
#include <rmagine/map/optix/optix_sbt.h>

// sensor connection
#include "CorrectionDataOptix.hpp"



namespace rmcl
{

struct CorrRayGenProgramGroup 
: public rmagine::ProgramGroup
{
    using RecordData        = rmagine::RayGenDataEmpty;
    using SbtRecordData     = rmagine::SbtRecord<RecordData>;

    SbtRecordData*        record_h      = nullptr;

    virtual ~CorrRayGenProgramGroup();
};

using CorrRayGenProgramGroupPtr = std::shared_ptr<CorrRayGenProgramGroup>;


struct CorrMissProgramGroup
: public rmagine::ProgramGroup
{
    using RecordData          = rmagine::MissDataEmpty;
    using SbtRecordData       = rmagine::SbtRecord<RecordData>;

    SbtRecordData*        record_h      = nullptr;

    virtual ~CorrMissProgramGroup();
};

using CorrMissProgramGroupPtr = std::shared_ptr<CorrMissProgramGroup>;



struct CorrHitProgramGroup
: public rmagine::ProgramGroup
, public rmagine::OptixSceneEventReceiver
{
    using RecordData        = rmagine::OptixSceneSBT;
    using SbtRecordData     = rmagine::SbtRecord<RecordData>;

    SbtRecordData*        record_h      = nullptr;

    virtual void onSBTUpdated(bool size_changed) override;

    virtual ~CorrHitProgramGroup();
};

using CorrHitProgramGroupPtr = std::shared_ptr<CorrHitProgramGroup>;


// GEN SW
CorrRayGenProgramGroupPtr make_program_group_corr_gen_sw(
    rmagine::OptixScenePtr scene,
    rmagine::ProgramModulePtr module);

CorrRayGenProgramGroupPtr make_program_group_corr_gen_sw(
    rmagine::OptixScenePtr scene,
    unsigned int sensor_id);

// MISS SW
CorrMissProgramGroupPtr make_program_group_corr_miss_sw(
    rmagine::OptixScenePtr scene,
    rmagine::ProgramModulePtr module);

CorrMissProgramGroupPtr make_program_group_corr_miss_sw(
    rmagine::OptixScenePtr scene);

// - Hit SW
CorrHitProgramGroupPtr make_program_group_corr_hit_sw(
    rmagine::OptixScenePtr scene,
    rmagine::ProgramModulePtr module);

CorrHitProgramGroupPtr make_program_group_corr_hit_sw(
    rmagine::OptixScenePtr scene);






CorrRayGenProgramGroupPtr make_program_group_corr_gen_rw(
    rmagine::OptixScenePtr scene,
    rmagine::ProgramModulePtr module);

CorrRayGenProgramGroupPtr make_program_group_corr_gen_rw(
    rmagine::OptixScenePtr scene,
    unsigned int sensor_id);

// MISS SW
CorrMissProgramGroupPtr make_program_group_corr_miss_rw(
    rmagine::OptixScenePtr scene,
    rmagine::ProgramModulePtr module);

CorrMissProgramGroupPtr make_program_group_corr_miss_rw(
    rmagine::OptixScenePtr scene);

// - Hit SW
CorrHitProgramGroupPtr make_program_group_corr_hit_rw(
    rmagine::OptixScenePtr scene,
    rmagine::ProgramModulePtr module);

CorrHitProgramGroupPtr make_program_group_corr_hit_rw(
    rmagine::OptixScenePtr scene);



} // namespace rmcl


#endif // RMCL_CORRECTION_OPTIX_CORR_PGS_H