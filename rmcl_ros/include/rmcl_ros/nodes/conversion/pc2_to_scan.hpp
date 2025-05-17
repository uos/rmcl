#ifndef RMCL_CONVERSION_PC2_TO_SCAN_HPP
#define RMCL_CONVERSION_PC2_TO_SCAN_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <rmcl_msgs/msg/scan_stamped.hpp>

#include <rmcl_ros/util/conversions.h>
#include <rmcl_ros/util/scan_operations.h>

#include <rmagine/math/types.h>
#include <rmagine/util/prints.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>
#include <tf2/convert.h>
#include <geometry_msgs/msg/transform_stamped.hpp>


namespace rm = rmagine;

namespace rmcl
{


class Pc2ToScanNode : public rclcpp::Node
{
public:
  explicit Pc2ToScanNode(
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:

  void fetchParameters();

  void initScanArray();

  bool convert(
      const sensor_msgs::msg::PointCloud2::ConstSharedPtr& pcd,
      rmcl_msgs::msg::ScanStamped& scan) const;

  void cloudCB(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg);

  std::string sensor_frame_ = "";
  bool debug_cloud_ = false;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcd_;
  
  rmcl_msgs::msg::ScanStamped scan_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pub_debug_cloud_;
  rclcpp::Publisher<rmcl_msgs::msg::ScanStamped>::SharedPtr pub_scan_;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
};

} // namespace rmcl

#endif // RMCL_CONVERSION_PC2_TO_SCAN_HPP