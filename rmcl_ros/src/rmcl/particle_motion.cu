#include "rmcl_ros/rmcl/particle_motion.cuh"
#include <cuda_runtime.h>

#include <iostream>

namespace rm = rmagine;

namespace rmcl
{

__global__
void particle_move_and_forget_kernel(
  rm::Transform* particle_poses, // in/out
  ParticleAttributes* particle_attrs, // in/out
  const rm::Transform T_bnew_bold,
  const double forget_rate,
  unsigned int N)
{
  const unsigned int pid = blockIdx.x * blockDim.x + threadIdx.x;
  if(pid < N)
  {
    // read data
    const rm::Transform pose_old = particle_poses[pid];
    ParticleAttributes attr = particle_attrs[pid];

    // apply motion and forget rates
    const rm::Transform pose_new = pose_old * T_bnew_bold;
    attr.likelihood.n_meas -= forget_rate * attr.likelihood.n_meas;
  
    // set changes
    particle_poses[pid] = pose_new;
    particle_attrs[pid] = attr;
  }
}

void particle_move_and_forget(
    rmagine::MemoryView<rmagine::Transform, rm::VRAM_CUDA> particle_poses_gpu, 
    rmagine::MemoryView<ParticleAttributes, rm::VRAM_CUDA> particle_attrs_gpu,
    const rmagine::Transform T_bnew_bold,
    const double forget_rate)
{
  constexpr unsigned int blockSize = 1024;
  const unsigned int gridSize = (particle_poses_gpu.size() + blockSize - 1) / blockSize;
  // move 100000000000 particles on the GPU and apply forgetting rate
  particle_move_and_forget_kernel<<<gridSize, blockSize>>>(particle_poses_gpu.raw(), particle_attrs_gpu.raw(), T_bnew_bold, forget_rate, particle_poses_gpu.size());
}


} // namespace rmcl