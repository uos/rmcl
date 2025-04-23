#ifndef RMCL_CORRECTION_MICP_SENSOR_CPU_HPP
#define RMCL_CORRECTION_MICP_SENSOR_CPU_HPP

#include <rmagine/math/types/CrossStatistics.hpp>
#include <rmagine/math/types/Transform.hpp>
#include <rmagine/types/Memory.hpp>
#include <rmcl_ros/correction/MICPSensor.hpp>

#include <rmcl/correction/SphereCorrectorEmbree.hpp>


namespace rmcl
{

/**
 * Implement this class to generate corrections 
 * in form of e.g. CrossStatistics per sensor
 * - output: CPU
 */
class MICPSensorCPU 
: public MICPSensor
{
public:
  MICPSensorCPU() {}

protected:
  // data, continuously filled by implementations (subclasses). e.g. topics
  rmagine::Memory<float, rmagine::RAM> ranges_;



};

} // namespace rmcl

#endif // RMCL_CORRECTION_MICP_SENSOR_CPU_HPP