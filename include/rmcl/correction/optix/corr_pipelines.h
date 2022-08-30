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