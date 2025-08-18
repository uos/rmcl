#ifndef RMCL_ROS_MCL_RESAMPLING_CUH
#define RMCL_ROS_MCL_RESAMPLING_CUH

/**
 * @brief collection of functions for resampling
 */

#include <rmagine/types/MemoryCuda.hpp>
#include <rmagine/types/shared_functions.h>

#include <rmcl_ros/rmcl/ParticleAttributes.hpp>

#include "Resampler.hpp"

#include <curand_kernel.h>
#include <curand.h>


namespace rmcl
{

void init_curand(rmagine::MemoryView<curandState, rmagine::VRAM_CUDA>& curand_states);

struct SimpleLikelihoodStats
{
  float sum = 0.0;
  float max = -1.0;
};

void compute_stats(
  const rmagine::MemoryView<rmagine::Transform, rmagine::VRAM_CUDA>& particle_poses,
  const rmagine::MemoryView<ParticleAttributes, rmagine::VRAM_CUDA>& particle_attrs,
  rmagine::MemoryView<SimpleLikelihoodStats, rmagine::VRAM_CUDA> stats);

void residual_resample(
  const rmagine::MemoryView<rmagine::Transform, rmagine::VRAM_CUDA> particle_poses,
  const rmagine::MemoryView<ParticleAttributes, rmagine::VRAM_CUDA> particle_attrs,
  const rmagine::MemoryView<SimpleLikelihoodStats, rmagine::VRAM_CUDA> stats,
  rmagine::MemoryView<curandState, rmagine::VRAM_CUDA> rstates,
  rmagine::MemoryView<rmagine::Transform, rmagine::VRAM_CUDA> particle_poses_new,
  rmagine::MemoryView<ParticleAttributes, rmagine::VRAM_CUDA> particle_attrs_new,
  const ParticleUpdateDynamicConfig& config);

} // namespace rmcl

#endif // RMCL_ROS_MCL_RESAMPLING_CUH