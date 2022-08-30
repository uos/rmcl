#ifndef RMCL_CORRECTION_OPTIX_CORR_MODULES_H
#define RMCL_CORRECTION_OPTIX_CORR_MODULES_H

// rmagine optix module interface
#include <rmagine/util/optix/optix_modules.h>

// map connection
#include <rmagine/map/optix/optix_definitions.h>

// sensor connection
#include "CorrectionDataOptix.hpp"



namespace rmcl
{

rmagine::ProgramModulePtr make_program_module_corr_sw(
    rmagine::OptixScenePtr scene,
    unsigned int sensor_id);

rmagine::ProgramModulePtr make_program_module_corr_rw(
    rmagine::OptixScenePtr scene,
    unsigned int sensor_id);

} // namespace rmcl

#endif // RMCL_CORRECTION_OPTIX_CORR_MODULES_H