#ifndef RMCL_FILTER_O1DN_MAP_SEGMENTATION_EMBREE_HPP
#define RMCL_FILTER_O1DN_MAP_SEGMENTATION_EMBREE_HPP

#include <rclcpp/rclcpp.hpp>

#include <rmcl_ros/nodes/filter/map_segmentation.hpp>

#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

// Rmagine deps
#include <rmagine/simulation/O1DnSimulatorEmbree.hpp>

// RCML msgs
#include <rmcl_msgs/msg/o1_dn_stamped.hpp>

#include <memory>

namespace rmcl
{

class O1DnMapSegmentationEmbreeNode 
: public MapSegmentationNode
{
public:
  explicit O1DnMapSegmentationEmbreeNode(
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  void scanCB(const rmcl_msgs::msg::O1DnStamped::ConstSharedPtr& msg) const;
private:
  rmagine::O1DnSimulatorEmbreePtr scan_sim_;
  
  rclcpp::Subscription<rmcl_msgs::msg::O1DnStamped>::SharedPtr sub_scan_;
};

} // namespace rmcl

#endif // RMCL_FILTER_O1DN_MAP_SEGMENTATION_EMBREE_HPP