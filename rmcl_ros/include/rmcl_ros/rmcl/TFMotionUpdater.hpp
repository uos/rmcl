#ifndef RMCL_MCL_TF_MOTION_UPDATER_HPP
#define RMCL_MCL_TF_MOTION_UPDATER_HPP

#include <memory>
#include <optional>

#include <rmagine/math/types.h>
#include <rmagine/types/Memory.hpp>

#include "ParticleAttributes.hpp"
#include "MotionUpdater.hpp"

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>

#include <rmagine/map/MapMap.hpp>


namespace rmcl
{

/**
 * @brief A 
 */
template<typename MemT>
class TFMotionUpdater 
: public MotionUpdater<MemT>
{
public:
  TFMotionUpdater(
    rmagine::MapMapPtr map_container,
    rclcpp::Node::SharedPtr node,
    std::shared_ptr<tf2_ros::Buffer> tf_buffer);

  void reset() override;

protected:
  rmagine::MapMapPtr map_container_;
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  std::string base_frame_;
  std::string odom_frame_;

  // motion update
  rmagine::Transform T_bold_o_;
  std::optional<rclcpp::Time> T_bold_o_stamp_;

  struct {
    double     weight_noise;
    double     forget_rate; // per meter
    double     forget_rate_per_second; // per second. TODO: move this to seperate "time_updater"
  } config_;
};

} // namespace rmcl

#include "TFMotionUpdater.tcc"

#endif // RMCL_MCL_TF_MOTION_UPDATER_HPP