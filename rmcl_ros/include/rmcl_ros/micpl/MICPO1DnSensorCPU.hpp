#ifndef RMCL_MICPO1DN_SENSOR_HPP
#define RMCL_MICPO1DN_SENSOR_HPP

#include <rclcpp/rclcpp.hpp>
#include <rmcl_ros/micpl/MICPSensor.hpp>
#include <rmcl_ros/micpl/MICPSensorCPU.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <rmcl_msgs/msg/o1_dn_stamped.hpp>

#include <rmagine/types/sensor_models.h>

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
#include <rmcl/registration/Correspondences.hpp>
#include <rmcl/registration/RCCEmbree.hpp>

#include <mutex>
#include <thread>
namespace rmcl
{

class MICPO1DnSensorCPU
: public MICPSensorCPU
{
public:

  using Base = MICPSensorCPU;

  MICPO1DnSensorCPU(
    rclcpp::Node::SharedPtr nh);

  // Data Loaders
  // TODO: Can we move this to seperate data loader instances?
  void connectToTopic(const std::string& topic_name);
  void getDataFromParameters();

  void updateMsg(const rmcl_msgs::msg::O1DnStamped::SharedPtr msg);
  
protected:

  void unpackMessage(const rmcl_msgs::msg::O1DnStamped::SharedPtr msg);

private:
  rmagine::O1DnModel sensor_model_;

  message_filters::Subscriber<rmcl_msgs::msg::O1DnStamped> data_sub_;
  std::unique_ptr<tf2_ros::MessageFilter<rmcl_msgs::msg::O1DnStamped> > tf_filter_;
};

} // namespace rmcl

#endif // RMCL_MICPO1DN_SENSOR_HPP