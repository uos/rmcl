#ifndef RMCL_RMCL_TOURNAMENT_RESAMPLER_CPU_HPP
#define RMCL_RMCL_TOURNAMENT_RESAMPLER_CPU_HPP

#include "Resampler.hpp"
#include "ParticleAttributes.hpp"

#include "GladiatorResamplerConfig.hpp"

#include <random>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>


namespace rmcl
{

class GladiatorResamplerCPU
: public Resampler<rmagine::RAM>
{
public:
  using Base = Resampler<rmagine::RAM>;

  GladiatorResamplerCPU(
    rclcpp::Node::SharedPtr node);

  void reset() override;

  void init() override;

  ParticleUpdateDynamicResults update(
    const rmagine::MemoryView<rmagine::Transform, rmagine::RAM> particle_poses,
    const rmagine::MemoryView<ParticleAttributes, rmagine::RAM> particle_attrs,
    rmagine::MemoryView<rmagine::Transform, rmagine::RAM> particle_poses_new,
    rmagine::MemoryView<ParticleAttributes, rmagine::RAM> particle_attrs_new,
    const ParticleUpdateDynamicConfig& config) override;

  // rmagine::Transform Tbm;

private:

  void updateParams();

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_runtime_;

  GladiatorResamplerConfig config_;

  std::unique_ptr<std::mt19937> rand_gen_;
};

} // namespace rmcl

#endif // RMCL_RMCL_TOURNAMENT_RESAMPLER_CPU_HPP