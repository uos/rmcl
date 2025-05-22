#ifndef RMCL_FILTER_MAP_SEGMENTATION_HPP
#define RMCL_FILTER_MAP_SEGMENTATION_HPP

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

// RCML msgs
#include <rmcl_msgs/msg/o1_dn_stamped.hpp>

#include <memory>

namespace rmcl
{

class MapSegmentationNode
: public rclcpp::Node
{
public:
  explicit MapSegmentationNode(
    std::string node_name,
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  rcl_interfaces::msg::SetParametersResult reconfigureCallback(
    const std::vector<rclcpp::Parameter>& params);

protected:

  float min_dist_outlier_scan_;
  float min_dist_outlier_map_;
  std::string map_frame_;
  std::string map_file_;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  
  rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pub_outlier_scan_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pub_outlier_map_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
};

} // namespace rmcl

#endif // RMCL_FILTER_MAP_SEGMENTATION_HPP