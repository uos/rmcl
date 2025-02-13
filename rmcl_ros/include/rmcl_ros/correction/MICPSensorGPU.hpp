#ifndef RMCL_CORRECTION_MICP_SENSOR_GPU_HPP
#define RMCL_CORRECTION_MICP_SENSOR_GPU_HPP

#include <rmagine/types/math/CrossStatistics.hpp>
#include <MICPSensor.hpp>

namespace rmcl
{

/**
 * Implement this class to generate corrections 
 * in form of e.g. CrossStatistics per sensor
 * - output: GPU
 */
class MICPSensorGPU : public MICPSensor
{
public:
    MICPSensorGPU() {}
};

} // namespace rmcl

#endif // RMCL_CORRECTION_MICP_SENSOR_GPU_HPP