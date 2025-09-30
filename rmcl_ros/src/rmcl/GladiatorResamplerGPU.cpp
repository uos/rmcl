#include <rmcl_ros/rmcl/GladiatorResamplerGPU.hpp>
#include <random>
#include <memory>
#include <rmagine/math/types.h>
#include <rmagine/util/StopWatch.hpp>
#include <rmcl_ros/util/ros_helper.h>

#include <rmcl_ros/rmcl/resampling.cuh>

namespace rm = rmagine;

namespace rmcl
{

GladiatorResamplerGPU::GladiatorResamplerGPU(
  rclcpp::Node::SharedPtr node)
:node_(node)
{
  std::random_device device{};
  rand_gen_ = std::make_unique<std::mt19937>(device());
  pub_runtime_ = node_->create_publisher<std_msgs::msg::Float64>("runtime", 10);
}

void GladiatorResamplerGPU::init()
{
  updateParams();
}

void GladiatorResamplerGPU::reset()
{

}

void GladiatorResamplerGPU::updateParams()
{
  config_.min_noise_tx = rmcl::get_parameter(node_, "~min_noise_tx", 0.03);
  config_.min_noise_ty = rmcl::get_parameter(node_, "~min_noise_ty", 0.03);
  config_.min_noise_tz = rmcl::get_parameter(node_, "~min_noise_tz", 0.0);
  
  config_.min_noise_roll  = rmcl::get_parameter(node_, "~min_noise_roll", 0.0);
  config_.min_noise_pitch = rmcl::get_parameter(node_, "~min_noise_pitch", 0.0);
  config_.min_noise_yaw   = rmcl::get_parameter(node_, "~min_noise_yaw", 0.01);

  config_.likelihood_forget_per_meter = rmcl::get_parameter(node_, "~likelihood_forget_per_meter", 0.3);
  config_.likelihood_forget_per_radian = rmcl::get_parameter(node_, "~likelihood_forget_per_radian", 0.2);
}

ParticleUpdateDynamicResults GladiatorResamplerGPU::update(
  const rmagine::MemoryView<rmagine::Transform, rmagine::VRAM_CUDA> particle_poses,
  const rmagine::MemoryView<ParticleAttributes, rmagine::VRAM_CUDA> particle_attrs,
  rmagine::MemoryView<rmagine::Transform, rmagine::VRAM_CUDA> particle_poses_new,
  rmagine::MemoryView<ParticleAttributes, rmagine::VRAM_CUDA> particle_attrs_new,
  const ParticleUpdateDynamicConfig& config)
{
  updateParams();

  if(rstates_.size() < particle_poses.size())
  {
    rstates_.resize(particle_poses.size());
    init_curand(rstates_);
  }

  rm::StopWatch sw;
  double el;

  ParticleUpdateDynamicResults res;

  // TODO: we already computed this on a subset of data
  rm::Memory<SimpleLikelihoodStats, rm::VRAM_CUDA> stats(1);

  sw();
  
  compute_stats(particle_poses, particle_attrs, stats);

  gladiator_resample(particle_poses, particle_attrs, 
      stats, rstates_, 
      particle_poses_new, particle_attrs_new, config_);

  res.n_particles = particle_poses_new.size();

  return res;
}

} // namespace rmcl