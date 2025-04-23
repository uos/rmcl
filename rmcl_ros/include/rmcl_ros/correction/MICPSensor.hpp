#ifndef RMCL_CORRECTION_MICP_SENSOR_HPP
#define RMCL_CORRECTION_MICP_SENSOR_HPP

#include <memory>
#include <string>

#include <rmagine/math/types/CrossStatistics.hpp>

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
class MICPSensor
{
public:
  MICPSensor() {}
  virtual ~MICPSensor() {}

  // name of the sensor
  std::string name;

  // Pipeline: load data -> search for correspondences -> update statistics
  // (later from base, for each sensor: merge statistics and compute pose corrections)




  // set a data source here. from topic file or similar. Put your own implementation here
  std::shared_ptr<DataLoader> data_loader;

  // model filled during correspondence search
  // std::shared_ptr<CorrespondenceFinder> model_corr;

  // reductor is a thing that reduces partial correspondences to an intermediate result per sensor
  // the results inside a reductor can be later merged in base frame
  // std::shared_ptr<Reductor> reductor;


};

using MICPSensorPtr = std::shared_ptr<MICPSensor>;

} // namespace rmcl


#endif // RMCL_CORRECTION_MICP_SENSOR_HPP