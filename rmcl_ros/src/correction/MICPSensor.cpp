#include "rmcl_ros/correction/MICPSensor.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rmcl_ros/util/conversions.h>

#include <rmcl_ros/util/ros_helper.h>




namespace rm = rmagine;

namespace rmcl
{

MICPSensorBase::MICPSensorBase(
  rclcpp::Node::SharedPtr nh)
:nh_(nh)
{
  tf_buffer_ =
    std::make_shared<tf2_ros::Buffer>(nh_->get_clock());

  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    nh_->get_node_base_interface(),
    nh_->get_node_timers_interface());
  tf_buffer_->setCreateTimerInterface(timer_interface);

  tf_listener_ =
    std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // load name
  ParamTree<rclcpp::Parameter>::SharedPtr sensor_param_tree
    = get_parameter_tree(nh, "~");

  // Get name of sensor: 'sensors.velodyne' -> 'velodyne'
  name = sensor_param_tree->name.substr(
    sensor_param_tree->name.find_last_of(".") + 1);

  correspondence_viz_pub_ = nh_->create_publisher<visualization_msgs::msg::Marker>("~/correspondences", 10);

  on_data_received = [](MICPSensorBase*){
    // default: dont use
  };
}

MICPSensorBase::~MICPSensorBase()
{
  
}

void MICPSensorBase::setTom(const rm::Transform& Tom_in)
{
  // when Tom changes, we have to find correspondences again
  Tom = Tom_in;
}

bool MICPSensorBase::fetchTF(const rclcpp::Time stamp)
{
  // figure out current transform chain.
  
  if(base_frame == sensor_frame)
  {
    Tsb_stamp = stamp;
    Tsb.setIdentity();
  } else {
    try {
      geometry_msgs::msg::TransformStamped T_sensor_base;
      T_sensor_base = tf_buffer_->lookupTransform(base_frame, sensor_frame, stamp);
      Tsb_stamp = T_sensor_base.header.stamp;
      convert(T_sensor_base.transform, Tsb);
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(nh_->get_logger(), "%s", ex.what());
      RCLCPP_WARN_STREAM(nh_->get_logger(), "Source (Sensor): " << sensor_frame << ", Target (Base): " << base_frame);
      return false;
    }
  }

  try {
    if(tf_buffer_->canTransform(
      odom_frame, base_frame, stamp, rclcpp::Duration::from_seconds(1.0)))
    {
      geometry_msgs::msg::TransformStamped T_base_odom;
      T_base_odom = tf_buffer_->lookupTransform(odom_frame, base_frame, stamp);
      Tbo_stamp = T_base_odom.header.stamp;
      convert(T_base_odom.transform, Tbo);
    }
    else
    {
      RCLCPP_WARN(nh_->get_logger(), "Transform not available yet.");
      return false;
    }
  } catch (tf2::TransformException& ex) {
    // std::cout << "Range sensor data is newer than odom! This not too bad. Try to get latest stamp" << std::endl;
    RCLCPP_WARN(nh_->get_logger(), "%s", ex.what());
    RCLCPP_WARN_STREAM(nh_->get_logger(), "Source (Base): " << base_frame << ", Target (Odom): " << odom_frame);
    return false;
  }

  // std::cout << "TRANSFORM CHAIN COMPLETE!" << std::endl;
  return true;
}

} // namespace rmcl