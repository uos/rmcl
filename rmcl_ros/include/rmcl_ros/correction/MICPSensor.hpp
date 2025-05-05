#ifndef RMCL_CORRECTION_MICP_SENSOR_HPP
#define RMCL_CORRECTION_MICP_SENSOR_HPP

#include <memory>
#include <string>

#include <rmagine/math/types/CrossStatistics.hpp>
#include <rmagine/types/Memory.hpp>

#include <rmcl_ros/correction/DataLoader.hpp>

namespace rmcl
{

/**
 * Implement this class to generate corrections 
 * in form of e.g. CrossStatistics per sensor
 * 
 * 
 * Rename this into "Pipleline?"
 * 
 */

template<typename MemT>
class MICPSensor_
{
public:
  MICPSensor_() {}
  virtual ~MICPSensor_() {}

  // name of the sensor
  std::string name;

  // Pipeline: load data -> search for correspondences -> update statistics
  // (later from base, for each sensor: merge statistics and compute pose corrections)

  std::shared_ptr<DataLoader> data_loader; // TODO: remove this


protected:
  // Data loader fills this (mutex?)
  rmagine::PointCloud_<MemT> dataset_;
  rclcpp::Time stamp_;
};

using MICPSensor = MICPSensor_<rmagine::RAM>;

using MICPSensorPtr = std::shared_ptr<MICPSensor>;

} // namespace rmcl


#endif // RMCL_CORRECTION_MICP_SENSOR_HPP