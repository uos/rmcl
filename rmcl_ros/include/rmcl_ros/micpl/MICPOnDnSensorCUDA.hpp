#ifndef RMCL_MICPL_ONDN_SENSOR_CUDA_HPP
#define RMCL_MICPL_ONDN_SENSOR_CUDA_HPP

#include <rclcpp/rclcpp.hpp>
#include <rmcl_ros/micpl/MICPSensorCUDA.hpp>

#include <rmcl_msgs/msg/on_dn_stamped.hpp>
#include <rmagine/types/sensor_models.h>

#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include "tf2_ros/transform_broadcaster.h"
#include <tf2_ros/buffer.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/create_timer_ros.h>

#include <message_filters/subscriber.h>

#include <rmagine/math/statistics.h>
#include <rmagine/types/MemoryCuda.hpp>

#include <mutex>
#include <thread>


namespace rmcl
{

class MICPOnDnSensorCUDA
: public MICPSensorCUDA
{
public:
  using Base = MICPSensorCUDA;

  MICPOnDnSensorCUDA(
    rclcpp::Node::SharedPtr nh);

  // Data Loaders
  // TODO: Can we move this to seperate data loader instances?
  void connectToTopic(const std::string& topic_name);
  void getDataFromParameters();

  void updateMsg(const rmcl_msgs::msg::OnDnStamped::SharedPtr msg);
  
protected:

  void unpackMessage(const rmcl_msgs::msg::OnDnStamped::SharedPtr msg);

private:

  rmagine::OnDnModel sensor_model_;

  message_filters::Subscriber<rmcl_msgs::msg::OnDnStamped> data_sub_;
  std::unique_ptr<tf2_ros::MessageFilter<rmcl_msgs::msg::OnDnStamped> > tf_filter_;
};

} // namespace rmcl

#endif // RMCL_MICPL_ONDN_SENSOR_CUDA_HPP