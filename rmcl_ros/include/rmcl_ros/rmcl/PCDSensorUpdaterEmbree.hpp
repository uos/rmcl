#ifndef RMCL_MCL_PCD_SENSOR_UPDATER_EMBREE_HPP
#define RMCL_MCL_PCD_SENSOR_UPDATER_EMBREE_HPP

#include <rmagine/types/Memory.hpp>
#include <rmagine/map/MapMap.hpp>

#include <rmagine/map/EmbreeMap.hpp>
#include "SensorUpdater.hpp"

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>

#include <std_msgs/msg/float64.hpp>

#include "Input.hpp"
#include "RangeMeasurement.hpp"


namespace rmcl
{

class PCDSensorUpdaterEmbree
: public SensorUpdater<rmagine::RAM>
, public Input<sensor_msgs::msg::PointCloud2::ConstSharedPtr>
{
public:
  using Base = SensorUpdater<rmagine::RAM>;

  PCDSensorUpdaterEmbree(
    rmagine::MapMapPtr map_container,
    rclcpp::Node::SharedPtr node,
    std::shared_ptr<tf2_ros::Buffer> tf_buffer);

  void init() override;

  void reset() override;

  ParticleUpdateResults update(
    rmagine::MemoryView<rmagine::Transform, rmagine::RAM> particle_poses,
    rmagine::MemoryView<ParticleAttributes, rmagine::RAM> particle_attrs,
    const ParticleUpdateConfig& config) override;

protected:

  /**
   * sensorUpdate for a single 1D measurement
   * 
   * Problem: state_uncertainty is multimodal, however, it converges against one mode (in the best case)
   * The particle belongs to one mode. Therefore I assume its OK. But I have no mathematical proof
   */
  ParticleAttributes sensorUpdate(
    const rmagine::Transform& Tsm,
    const RangeMeasurement& meas_s,
    const ParticleAttributes& particle_attr
  ) const;

private:

  void updateParams();

  void loadMap();

  rmagine::MapMapPtr map_container_;
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_runtime_;

  std::string base_frame_;

  rmagine::EmbreeMapPtr map_;
  struct {
    // sensor update
    // Correspondence type
    // 0: RCC
    // 1: NN
    unsigned int correspondence_type = 0;
    
    unsigned int samples = 10;

    // certainty about measurement (+pose?)
    // normally this is a value which adds up state and measurement uncertainty.
    // not implemented yet, but a wild idea:
    // - give only a parameter for measurement uncertainty here
    // - automatically detect state uncertainty from the particles
    // - add both together.
    // -> this would result in a dynamically adjusted certainty based on the current state's certainty
    float dist_sigma = 2.0;

    // exceptions
    float real_hit_sim_miss_error; // in meter
    float real_miss_sim_hit_error; // in meter
    float real_miss_sim_miss_error; // in meter

    rmagine::Interval sensor_range;
  } config_;

};

} // namespace rmcl

#endif // RMCL_MCL_PCD_SENSOR_UPDATER_EMBREE_HPP
