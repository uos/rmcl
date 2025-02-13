#ifndef RMCL_CORRECTION_MICP_SENSOR_CPU_HPP
#define RMCL_CORRECTION_MICP_SENSOR_CPU_HPP

#include <rmagine/types/math/CrossStatistics.hpp>
#include <MICPSensor.hpp>

namespace rmcl
{

/**
 * Implement this class to generate corrections 
 * in form of e.g. CrossStatistics per sensor
 * - output: CPU
 */
class MICPSensorCPU : MICPSensor 
{
public:
    MICPSensorCPU() {}
};

} // namespace rmcl

#endif // RMCL_CORRECTION_MICP_SENSOR_CPU_HPP