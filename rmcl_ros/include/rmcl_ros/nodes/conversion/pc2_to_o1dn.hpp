#ifndef RMCL_CONVERSION_PC2_TO_O1DN_HPP
#define RMCL_CONVERSION_PC2_TO_O1DN_HPP

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <rmcl_msgs/msg/o1_dn_stamped.hpp>
#include <rmcl_ros/util/scan_operations.h>
#include <rmcl_ros/util/conversions.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>
#include <tf2/convert.h>
#include <geometry_msgs/msg/transform_stamped.hpp>


namespace rmcl
{

class Pc2ToO1DnNode : public rclcpp::Node
{
public:
  explicit Pc2ToO1DnNode(
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:

  void declareParameters();

  void fetchParameters();

  rcl_interfaces::msg::SetParametersResult parametersCallback(
    const std::vector<rclcpp::Parameter> &parameters);

  bool convert(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& pcd,
    rmcl_msgs::msg::O1DnStamped& scan) const;

  void cloudCB(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg);

  // parameters
  std::string sensor_frame_ = "";
  bool debug_cloud_ = false;

  FilterOptions2D filter_options_;
  
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcl_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_debug_cloud_;
  rclcpp::Publisher<rmcl_msgs::msg::O1DnStamped>::SharedPtr pub_scan_;
  rmcl_msgs::msg::O1DnStamped scan_;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  OnSetParametersCallbackHandle::SharedPtr callback_handle_;
};

} // namespace rmcl

#endif // RMCL_CONVERSION_PC2_TO_O1DN_HPP