#ifndef RMCL_MCL_RESIDUAL_RESAMPLER_CPU_HPP
#define RMCL_MCL_RESIDUAL_RESAMPLER_CPU_HPP

#include "Resampler.hpp"
#include "ParticleAttributes.hpp"

#include <random>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>


namespace rmcl
{

class ResidualResamplerCPU
: public Resampler<rmagine::RAM>
{
public:
  using Base = Resampler<rmagine::RAM>;

  ResidualResamplerCPU(
    rclcpp::Node::SharedPtr node);

  void reset() override;

  void init() override;

  ParticleUpdateDynamicResults update(
    const rmagine::MemoryView<rmagine::Transform, rmagine::RAM> particle_poses,
    const rmagine::MemoryView<ParticleAttributes, rmagine::RAM> particle_attrs,
    rmagine::MemoryView<rmagine::Transform, rmagine::RAM> particle_poses_new,
    rmagine::MemoryView<ParticleAttributes, rmagine::RAM> particle_attrs_new,
    const ParticleUpdateDynamicConfig& config) override;

private:

  void updateParams();

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_runtime_;

  struct {
    // TODO: add config here

    float min_noise_tx; // motion + sensor noise
    float min_noise_ty;
    float min_noise_tz;
    float min_noise_roll;
    float min_noise_pitch;
    float min_noise_yaw;

    // dependent how far from the original sample 
    float likelihood_forget_per_meter = 0.3;
    float likelihood_forget_per_radian = 0.2;

  } config_;

  std::unique_ptr<std::mt19937> rand_gen_;
};

} // namespace rmcl

#endif // RMCL_MCL_RESIDUAL_RESAMPLER_CPU_HPP