#ifndef RMCL_CONVERSION_SCAN_TO_SCAN_HPP
#define RMCL_CONVERSION_SCAN_TO_SCAN_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
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

class ScanToScanNode : public rclcpp::Node
{
public:
  explicit ScanToScanNode(
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:

  void declareParameters();

  void fetchParameters();

  rcl_interfaces::msg::SetParametersResult parametersCallback(
    const std::vector<rclcpp::Parameter> &parameters);

  void initScanArray();

  bool convert(
      const sensor_msgs::msg::LaserScan::ConstSharedPtr& scan_in,
      rmcl_msgs::msg::ScanStamped& scan_out) const;

  void scanCB(const sensor_msgs::msg::LaserScan::ConstSharedPtr& msg);

  size_t skip_begin_;
  size_t skip_end_;
  size_t increment_;
  bool debug_cloud_ = false;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;

  rmcl_msgs::msg::ScanStamped scan_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pub_debug_cloud_;
  rclcpp::Publisher<rmcl_msgs::msg::ScanStamped>::SharedPtr pub_scan_;
  
  OnSetParametersCallbackHandle::SharedPtr callback_handle_;
};

} // namespace rmcl

#endif // RMCL_CONVERSION_PC2_TO_SCAN_HPP