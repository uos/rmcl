#ifndef RMCL_ROS_MCL_PARTICLE_MOTION_CUH
#define RMCL_ROS_MCL_PARTICLE_MOTION_CUH

/**
 * @brief collection of functions helping the particles move on the GPU
 */

#include <rmagine/types/MemoryCuda.hpp>
#include <rmagine/types/shared_functions.h>

#include <rmcl_ros/rmcl/ParticleAttributes.hpp>

namespace rmcl
{

void particle_move_and_forget(
  rmagine::MemoryView<rmagine::Transform, rmagine::VRAM_CUDA> particle_poses_gpu, 
  rmagine::MemoryView<ParticleAttributes, rmagine::VRAM_CUDA> particle_attrs_gpu,
  const rmagine::Transform T_bnew_bold,
  const double forget_rate);

} // namespace rmcl

#endif // RMCL_ROS_MCL_PARTICLE_MOTION_CUH