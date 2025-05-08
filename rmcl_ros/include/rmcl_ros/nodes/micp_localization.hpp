#ifndef RMCL_ROS_NODES_MICP_LOCALIZATION_HPP
#define RMCL_ROS_NODES_MICP_LOCALIZATION_HPP

#include <rclcpp/rclcpp.hpp>

#include <rmcl_ros/util/ros_helper.h>
#include <rmcl_ros/correction/MICPSensor.hpp>
#include <rmcl_ros/correction/DataLoader.hpp>

#include <unordered_map>
#include <string>
#include <thread>
#include <memory>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/create_timer_ros.h>
#include <message_filters/subscriber.h>

#include <rmagine/map/EmbreeMap.hpp>


namespace rmcl
{

class MICPLocalizationNode : public rclcpp::Node
{
public:
  explicit MICPLocalizationNode(
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  /**
   * Loads a single sensor config
   */
  MICPSensorPtr loadSensor(
      ParamTree<rclcpp::Parameter>::SharedPtr sensor_params);

  
  // I make this public if someone wants to write an application 
  // for multiple robots
  rmagine::Transform Tom;
  rclcpp::Time Tom_stamp;

private:
  void poseCB(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

  void sensorDataReceived(const MICPSensorBase* sensor);

  void correctionLoop();

  std::string map_frame_;
  std::string base_frame_;
  std::string odom_frame_;
  bool        use_odom_frame_;

  std::string map_filename_;
  std::unordered_map<std::string, MICPSensorPtr> sensors_;

  rmagine::EmbreeMapPtr map_embree_;

  // pose wc stamped subscriber (eg, RViz)
  // rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;

  
  // rclcpp::TimerBase::SharedPtr correction_timer_;

  std::thread correction_thread_;
  bool stop_correction_thread_ = false;

  // tf2
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;


  // pose wc stamped subscriber (eg, RViz)
  message_filters::Subscriber<geometry_msgs::msg::PoseWithCovarianceStamped> pose_sub_;
  std::unique_ptr<tf2_ros::MessageFilter<geometry_msgs::msg::PoseWithCovarianceStamped> > pose_tf_filter_;

  

  // double max_tf_rate = 10.0;
  // rclcpp::Time last_correction_stamp; // last correction time
};


} // namespace rmcl

#endif // RMCL_ROS_NODES_MICP_LOCALIZATION_HPP