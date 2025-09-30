#include "TFMotionUpdater.hpp"
#include <iostream>
#include <rmagine/math/types/Transform.hpp>
#include <rmcl_ros/util/ros_helper.h>

namespace rmcl
{

template<typename MemT>
TFMotionUpdater<MemT>::TFMotionUpdater(
  rmagine::MapMapPtr map_container,
  rclcpp::Node::SharedPtr node,
  std::shared_ptr<tf2_ros::Buffer> tf_buffer)
:map_container_(map_container)
,node_(node)
,tf_buffer_(tf_buffer)
{
  std::cout << "Init TFMotionUpdater" << std::endl;
  T_bold_o_ = rmagine::Transform::Identity();
  base_frame_ = rmcl::get_parameter(node_, "base_frame", "base_link");
  odom_frame_ = rmcl::get_parameter(node_, "odom_frame", "odom");
}

template<typename MemT>
void TFMotionUpdater<MemT>::reset()
{
  T_bold_o_stamp_.reset();
}

} // namespace rmcl