#ifndef RMCL_CORRECTION_MICP_SENSOR_HPP
#define RMCL_CORRECTION_MICP_SENSOR_HPP

#include <rclcpp/rclcpp.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include "tf2_ros/transform_broadcaster.h"
#include <tf2_ros/buffer.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/create_timer_ros.h>

#include <memory>
#include <string>

#include <rmagine/math/types/CrossStatistics.hpp>
#include <rmagine/types/Memory.hpp>

#include <rmcl_ros/correction/Correspondences.hpp>

namespace rmcl
{

  
// class MICPSensor
// {
// public:
//   MICPSensor() {}
//   virtual ~MICPSensor() {}

//   // name of the sensor
//   std::string name;
// };

class MICPSensorBase
{
public:
  MICPSensorBase(rclcpp::Node::SharedPtr nh);
  virtual ~MICPSensorBase();

  // Pipeline: load data -> search for correspondences -> update statistics
  // (later from base, for each sensor: merge statistics and compute pose corrections)

  void setTbm(const rmagine::Transform& Tbm);

  /**
   * Fetch TF chain. except for odom->map (this is estimated or has to be provided from extern)
   */
  void fetchTF();

  // name of the sensor
  std::string name;

  // transform chain from sensor -> base -> odom -> map

  // keep this up to date
  rmagine::Transform Tsb;
  rclcpp::Time Tsb_stamp;
  rmagine::Transform Tbo;
  rclcpp::Time Tbo_stamp;
  rmagine::Transform Tom;
  rclcpp::Time Tom_stamp;

  
  std::string map_frame = "map";
  std::string odom_frame = "odom";
  std::string base_frame = "base_footprint";
  std::string sensor_frame = "velodyne";

  rclcpp::Time dataset_stamp_;

  // ROS
  rclcpp::Node::SharedPtr nh_;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

template<typename MemT>
class MICPSensor_
: public MICPSensorBase
{
public:
  MICPSensor_(rclcpp::Node::SharedPtr nh)
  :MICPSensorBase(nh)
  {

  }

  std::shared_ptr<Correspondences_<MemT> > correspondences_;

protected:
  // Data loader fills this (mutex?)
  rmagine::PointCloud_<MemT> dataset_;
};

using MICPSensor = MICPSensor_<rmagine::RAM>;

using MICPSensorPtr = std::shared_ptr<MICPSensor>;

} // namespace rmcl


#endif // RMCL_CORRECTION_MICP_SENSOR_HPP