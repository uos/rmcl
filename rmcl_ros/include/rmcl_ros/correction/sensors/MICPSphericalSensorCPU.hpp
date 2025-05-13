#ifndef RMCL_MICP_SPHERICAL_SENSOR_HPP
#define RMCL_MICP_SPHERICAL_SENSOR_HPP

#include <rclcpp/rclcpp.hpp>
#include <rmcl_ros/correction/MICPSensor.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <rmcl_msgs/msg/scan_stamped.hpp>

#include <rmagine/types/sensor_models.h>
#include <rmagine/simulation/O1DnSimulatorEmbree.hpp>

#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include "tf2_ros/transform_broadcaster.h"
#include <tf2_ros/buffer.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/create_timer_ros.h>

#include <message_filters/subscriber.h>


#include <rmagine/math/statistics.h>
#include <rmagine/math/linalg.h>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

// Correspondences
#include <rmcl_ros/correction/Correspondences.hpp>
#include <rmcl_ros/correction/correspondences/RCCEmbree.hpp>


#include <mutex>
#include <thread>

namespace rmcl
{

class MICPSphericalSensorCPU
: public MICPSensor_<rmagine::RAM>
{
public:
  using Base = MICPSensor_<rmagine::RAM>;

  MICPSphericalSensorCPU(
    rclcpp::Node::SharedPtr nh,
    std::string topic_name);

  void topicCB(const rmcl_msgs::msg::ScanStamped::SharedPtr msg);
  
protected:

  void unpackMessage(const rmcl_msgs::msg::ScanStamped::SharedPtr msg);

private:
  rmagine::SphericalModel sensor_model_;

  message_filters::Subscriber<rmcl_msgs::msg::ScanStamped> data_sub_;
  std::unique_ptr<tf2_ros::MessageFilter<rmcl_msgs::msg::ScanStamped> > tf_filter_;
};

} // namespace rmcl

#endif // RMCL_MICP_SPHERICAL_SENSOR_HPP