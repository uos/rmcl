#ifndef RMCL_FILTER_SCAN_MAP_SEGMENTATION_EMBREE_HPP
#define RMCL_FILTER_SCAN_MAP_SEGMENTATION_EMBREE_HPP

#include <rclcpp/rclcpp.hpp>

#include <rmcl_ros/nodes/filter/map_segmentation.hpp>

#include <sensor_msgs/msg/point_cloud.hpp>

// Rmagine deps
#include <rmagine/simulation/SphereSimulatorEmbree.hpp>

// RCML msgs
#include <rmcl_msgs/msg/scan_stamped.hpp>

#include <memory>

namespace rmcl
{

class ScanMapSegmentationEmbreeNode 
: public MapSegmentationNode
{
public:
  explicit ScanMapSegmentationEmbreeNode(
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  void scanCB(const rmcl_msgs::msg::ScanStamped::ConstSharedPtr& msg) const;

private:

  rmagine::SphereSimulatorEmbreePtr scan_sim_;
  rclcpp::Subscription<rmcl_msgs::msg::ScanStamped>::SharedPtr sub_scan_;
};

} // namespace rmcl

#endif // RMCL_FILTER_SCAN_MAP_SEGMENTATION_EMBREE_HPP