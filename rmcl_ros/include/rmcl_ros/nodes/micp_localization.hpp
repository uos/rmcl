#ifndef RMCL_ROS_NODES_MICP_LOCALIZATION_HPP
#define RMCL_ROS_NODES_MICP_LOCALIZATION_HPP

#include <rclcpp/rclcpp.hpp>

#include <rmcl_ros/util/ros_helper.h>
#include <rmcl_ros/micpl/MICPSensor.hpp>

#include <unordered_map>
#include <string>
#include <thread>
#include <memory>
#include <mutex>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/create_timer_ros.h>
#include <message_filters/subscriber.h>


#include <rmagine/map/MapMap.hpp>

#ifdef RMCL_EMBREE
#include <rmagine/map/EmbreeMap.hpp>
#endif // RMCL_EMBREE

#ifdef RMCL_OPTIX
#include <rmagine/map/OptixMap.hpp>
#endif // RMCL_OPTIX

#include <rmcl_msgs/msg/micp_stats.hpp>
#include <rmcl_msgs/msg/micp_sensor_stats.hpp>

#include <mutex>
#include <atomic>

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

  void printSetup();

  // I make this public if someone wants to write an application 
  // for multiple robots
  rmagine::Transform Tom_;
  rclcpp::Time Tom_stamp_;

private:

  void poseCB(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

  void sensorDataReceived(const MICPSensorBase* sensor);

  void correct();

  void correctOnce();

  void correctionLoop();

  void broadcastTransform();

  void publishPose();

  bool fetchTF(const rclcpp::Time stamp);

  void tfBroadcastLoop();

  std::string map_frame_;
  std::string base_frame_;
  std::string odom_frame_;

  std::string map_filename_;
  std::unordered_map<std::string, MICPSensorPtr> sensors_;
  std::vector<MICPSensorPtr> sensors_vec_;

  // TODO: use this to become independent from the implementations 
  // at this place in code
  // rmagine::MapMap map_server_;

  // TODO: can we avoid ifdefs here?
  #ifdef RMCL_EMBREE
  rmagine::EmbreeMapPtr map_embree_;
  #endif // RMCL_EMBREE
  
  #ifdef RMCL_OPTIX
  rmagine::OptixMapPtr  map_optix_;
  #endif // RMCL_OPTIX
  
  std::thread correction_thread_;
  bool stop_correction_thread_ = false;

  std::thread tf_broadcaster_thread_;
  bool stop_tf_broadcaster_thread_ = false;

  // tf2
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // pose wc stamped subscriber (eg, RViz)
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr 
    pose_sub_;
  
  rclcpp::Publisher<rmcl_msgs::msg::MICPSensorStats>::SharedPtr 
    stats_publisher_;

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr 
    Tbm_publisher_;
  
  std::mutex mutex_;

  // High-Level stats about sensors
  rclcpp::Time data_stamp_latest_;
  rmagine::Transform Tbo_latest_;
  rclcpp::Time Tbo_stamp_latest_;
  rmcl_msgs::msg::MICPSensorStats correction_stats_latest_;
  double convergence_progress_ = 0.0;
  bool first_message_received_ = false;

  // IF there are at least 1 dynamic sensors (reading data from topic):
  //    synchronize all static sensors with the latest dynamic message
  //    to get small time errors. 
  // ELSE 
  //    It is OK to synchronize with the current time. 
  size_t num_dynamic_sensors_ = 0;
  
  bool disable_correction_ = false;
  size_t optimization_iterations_ = 10;
  int tf_time_source_ = 0;
  double pose_noise_ = 0.01;
  bool broadcast_tf_ = true;
  bool publish_pose_ = false;
  bool adaptive_max_dist_ = true;
  double correction_rate_max_ = 100.0;
  rmagine::Transform initial_pose_offset_;

  double tf_rate_ = 100.0;
};


} // namespace rmcl

#endif // RMCL_ROS_NODES_MICP_LOCALIZATION_HPP
