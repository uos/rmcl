#ifndef RMCL_ROS_NODES_MICP_LOCALIZATION_HPP
#define RMCL_ROS_NODES_MICP_LOCALIZATION_HPP

#include <rclcpp/rclcpp.hpp>

#include <rmcl_ros/util/ros_helper.h>
#include <rmcl_ros/correction/MICPSensor.hpp>
#include <rmcl_ros/correction/DataLoader.hpp>

#include <unordered_map>
#include <string>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

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

private:
  void poseCB(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

  std::string map_frame_;
  std::string base_frame_;
  std::string odom_frame_;
  bool        use_odom_frame_;

  std::string map_filename_;
  std::unordered_map<std::string, MICPSensorPtr> sensors_;

  rmagine::EmbreeMapPtr map_embree_;

  // pose wc stamped subscriber (eg, RViz)
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
};


} // namespace rmcl

#endif // RMCL_ROS_NODES_MICP_LOCALIZATION_HPP