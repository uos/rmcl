#include <rmcl_ros/rmcl/ResidualResamplerGPU.hpp>
#include <random>
#include <memory>
#include <rmagine/math/types.h>
#include <rmagine/util/StopWatch.hpp>
#include <rmcl_ros/util/ros_helper.h>

#include <rmcl_ros/rmcl/resampling.cuh>

namespace rm = rmagine;

namespace rmcl
{

ResidualResamplerGPU::ResidualResamplerGPU(
  rclcpp::Node::SharedPtr node)
:node_(node)
{
  std::random_device device{};
  rand_gen_ = std::make_unique<std::mt19937>(device());
  pub_runtime_ = node_->create_publisher<std_msgs::msg::Float64>("runtime", 10);
}

void ResidualResamplerGPU::init()
{
  updateParams();
}

void ResidualResamplerGPU::reset()
{

}

void ResidualResamplerGPU::updateParams()
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

ParticleUpdateDynamicResults ResidualResamplerGPU::update(
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


  // rm::Memory<ParticleAttributes, rm::RAM> particle_attrs_cpu = particle_attrs;

  // float max = 0.0;
  // float sum = 0.0;

  // for(size_t i=0; i<particle_attrs_cpu.size(); i++)
  // {
  //   const float L = particle_attrs_cpu[i].likelihood.mean;
  //   max = std::max(L, max);
  //   sum += L;
  // }

  // std::cout << "CPU Stats:" << std::endl;
  // std::cout << "- sum: " << sum << std::endl;
  // std::cout << "- max: " << max << std::endl;
  

  ParticleUpdateDynamicResults res;

  rm::Memory<SimpleLikelihoodStats, rm::VRAM_CUDA> stats(1);

  sw();
  
  compute_stats(particle_poses, particle_attrs, stats);

  // rm::Memory<SimpleLikelihoodStats, rm::RAM> stats_cpu = stats;
  // std::cout << "GPU Stats:" << std::endl; 
  // std::cout << "- sum: " << stats_cpu[0].sum << std::endl;
  // std::cout << "- max: " << stats_cpu[0].max << std::endl;

  residual_resample(particle_poses, particle_attrs, 
      stats, rstates_, 
      particle_poses_new, particle_attrs_new, config);

  res.n_particles = particle_poses_new.size();

  el = sw();

  std::cout << "   - runtime: " << el << "s" << std::endl;


  return res;
}

} // namespace rmcl