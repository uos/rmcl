#ifndef RMCL_MCL_TOURNAMENT_RESAMPLER_GPU_HPP
#define RMCL_MCL_TOURNAMENT_RESAMPLER_GPU_HPP

#include "Resampler.hpp"
#include "ParticleAttributes.hpp"
#include "GladiatorResamplerConfig.hpp"

#include <random>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

#include <rmagine/types/MemoryCuda.hpp>

#include <curand.h>
#include <curand_kernel.h>

namespace rmcl
{

class GladiatorResamplerGPU
: public Resampler<rmagine::VRAM_CUDA>
{
public:
  using Base = Resampler<rmagine::VRAM_CUDA>;

  GladiatorResamplerGPU(
    rclcpp::Node::SharedPtr node);

  void reset() override;

  void init() override;

  ParticleUpdateDynamicResults update(
    const rmagine::MemoryView<rmagine::Transform, rmagine::VRAM_CUDA> particle_poses,
    const rmagine::MemoryView<ParticleAttributes, rmagine::VRAM_CUDA> particle_attrs,
    rmagine::MemoryView<rmagine::Transform, rmagine::VRAM_CUDA> particle_poses_new,
    rmagine::MemoryView<ParticleAttributes, rmagine::VRAM_CUDA> particle_attrs_new,
    const ParticleUpdateDynamicConfig& config) override;

private:

  void updateParams();

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_runtime_;

  GladiatorResamplerConfig config_;

  std::unique_ptr<std::mt19937> rand_gen_;

  rmagine::Memory<curandState, rmagine::VRAM_CUDA> rstates_;
  
};

} // namespace rmcl

#endif // RMCL_MCL_TOURNAMENT_RESAMPLER_GPU_HPP