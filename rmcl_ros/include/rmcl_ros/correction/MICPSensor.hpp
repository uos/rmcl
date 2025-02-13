#ifndef RMCL_CORRECTION_MICP_SENSOR_HPP
#define RMCL_CORRECTION_MICP_SENSOR_HPP

#include <rmagine/types/math/CrossStatistics.hpp>

namespace rmcl
{

/**
 * Implement this class to generate corrections 
 * in form of e.g. CrossStatistics per sensor
 */
class MICPSensor
{
public:
    MICPSensor() {}
    virtual ~MICPSensor() {}
};

} // namespace rmcl

#endif // RMCL_CORRECTION_MICP_SENSOR_HPP