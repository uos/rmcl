#include "rmcl_ros/rmcl/resampling.cuh"
#include <cuda_runtime.h>

#include <iostream>

#include <curand.h>

namespace rm = rmagine;

namespace rmcl
{

__global__ 
void init_curand_kernel(curandState* states, const unsigned int N)
{
  unsigned int idx = threadIdx.x + blockDim.x * blockIdx.x;

  if(idx < N)
  {
    curand_init(1234, idx, 0, &states[idx]);
  }
}

void init_curand(rmagine::MemoryView<curandState, rmagine::VRAM_CUDA>& curand_states)
{
  constexpr unsigned int blockSize = 1024;
  const unsigned int gridSize = (curand_states.size() + blockSize - 1) / blockSize;

  init_curand_kernel<<<gridSize, blockSize>>>(curand_states.raw(), curand_states.size());
}

__device__
SimpleLikelihoodStats merge(const SimpleLikelihoodStats s1, const SimpleLikelihoodStats s2)
{
  SimpleLikelihoodStats sm;
  sm.sum = s1.sum + s2.sum;
  sm.max = max(s1.max, s2.max);
  return sm;
}

template<unsigned int blockSize>
__global__ void simple_stats_kernel(
    const rm::Transform* poses,
    const ParticleAttributes* attrs,
    const unsigned int N,
    SimpleLikelihoodStats* res)
{
  __shared__ SimpleLikelihoodStats sdata[blockSize];
  
  const unsigned int tid = threadIdx.x;
  const unsigned int globId = N * blockIdx.x + threadIdx.x;
  const unsigned int rows = (N + blockSize - 1) / blockSize;

  sdata[tid].sum = 0.0; // TODO: this is a trick, but not good
  sdata[tid].max = 0.0;

  for(unsigned int i=0; i<rows; i++)
  {
    if(globId + blockSize * i < N)
    {
      const float L = attrs[globId + blockSize * i].likelihood.mean;
      sdata[threadIdx.x].sum += L;
      sdata[tid].max = max(sdata[tid].max, L);
    }
  }
  __syncthreads();

  for(unsigned int s = blockSize / 2; s > 0; s >>= 1)
  {
    if(tid < s)
    {
      sdata[tid] = merge(sdata[tid], sdata[tid + s]);
    }
    __syncthreads();
  }

  if(tid == 0)
  {
    res[blockIdx.x] = sdata[0];
  }
}

void compute_stats(
  const rm::MemoryView<rm::Transform, rmagine::VRAM_CUDA>& particle_poses,
  const rm::MemoryView<ParticleAttributes, rmagine::VRAM_CUDA>& particle_attrs,
  rm::MemoryView<SimpleLikelihoodStats, rmagine::VRAM_CUDA> stats)
{
  const unsigned int n_outputs = stats.size(); // also number of blocks
  constexpr unsigned int n_threads = 512; // also shared mem

  simple_stats_kernel<n_threads> <<<n_outputs, n_threads>>>(particle_poses.raw(), particle_attrs.raw(), particle_poses.size(), stats.raw());
}


__device__ unsigned int lcg_rand(unsigned int &state) {
  // Constants from Numerical Recipes
  const unsigned int A = 1664525;
  const unsigned int C = 1013904223;
  state = A * state + C;
  return state;
}

__device__ float lcg_rand_flt(unsigned int& state) {
  return lcg_rand(state) / (float)UINT_MAX;
}


__global__
void tournament_resample_kernel(
  const rmagine::Transform* particle_poses,
  const ParticleAttributes* particle_attrs,
  const SimpleLikelihoodStats* stats,
  curandState* rstates,
  rmagine::Transform* particle_poses_new,
  ParticleAttributes* particle_attrs_new,
  const unsigned int n_particles,
  const ParticleUpdateDynamicConfig config)
{
  const unsigned int pid = blockIdx.x * blockDim.x + threadIdx.x;

  const float min_noise_tx = 0.1;
  const float min_noise_ty = 0.1;
  const float min_noise_tz = 0.05;
  
  const float min_noise_roll  = 0.0;
  const float min_noise_pitch = 0.0;
  const float min_noise_yaw   = 0.05;

  const float likelihood_forget_per_meter = 0.3;
  const float likelihood_forget_per_radian = 0.2;

  const float L_max = stats->max;
  const float L_sum = stats->sum;

  if(pid < n_particles)
  {
    // we need to do this reversed than the CPU version:
    // 1. One thread is handling one insertion index
    // 2. Get a random other element and see if it is 
    //    effecting the current element

    const unsigned int insertion_idx = pid;
    
    // Get random number
    curandState& rstate = rstates[pid];
    const unsigned int random_int = curand(&rstate);
    const unsigned int random_idx = random_int % n_particles;
    const float random_flt = (random_int / (float)UINT_MAX); // betwen 0 and 1
    const float Nd_tx = curand_normal(&rstate);
    const float Nd_ty = curand_normal(&rstate);
    const float Nd_tz = curand_normal(&rstate);
    const float Nd_rx = curand_normal(&rstate);
    const float Nd_ry = curand_normal(&rstate);
    const float Nd_rz = curand_normal(&rstate);
    

    const unsigned int source_idx = random_idx;

    const float Ls = particle_attrs[source_idx].likelihood.mean;
    const float Ls_max_normed = Ls / L_max;
    const float Li = particle_attrs[insertion_idx].likelihood.mean;
    const float Li_max_normed = Li / L_max;

    rm::Transform pose;
    ParticleAttributes attrs;

    if(Ls > Li)
    {
      // source pose is winner!
      pose = particle_poses[source_idx];
      attrs = particle_attrs[source_idx];
    } else {
      // insertion (target) pose is winner!
      pose = particle_poses[insertion_idx];
      attrs = particle_attrs[insertion_idx];
    }

    rm::Transform pose_new = pose;
    ParticleAttributes attrs_new = attrs;

    const float noise_tx = min_noise_tx;
    const float noise_ty = min_noise_ty;
    const float noise_tz = min_noise_tz;

    const float noise_roll  = min_noise_roll;
    const float noise_pitch = min_noise_pitch;
    const float noise_yaw   = min_noise_yaw;

    pose_new.t.x += Nd_tx * noise_tx;
    pose_new.t.y += Nd_ty * noise_ty;
    pose_new.t.z += Nd_tz * noise_tz;
    rm::EulerAngles e = pose_new.R;
    e.roll += Nd_rx * noise_roll;
    e.pitch += Nd_ry * noise_pitch;
    e.yaw += Nd_rz * noise_yaw;
    pose_new.R = e;

    const rm::Transform pose_diff = ~pose * pose_new;

    // keep the likelihood, reduce number of measurements
    const float trans_dist = pose_diff.t.l2normSquared(); // in meter
    const float rot_dist = pose_diff.R.l2norm(); // in radian
    const float reduction_factor = pow(likelihood_forget_per_meter, trans_dist) 
                                * pow(likelihood_forget_per_radian, rot_dist);
    attrs_new.likelihood.n_meas *= reduction_factor;

    particle_poses_new[insertion_idx] = pose_new;
    particle_attrs_new[insertion_idx] = attrs_new;
  }
}

void tournament_resample(
  const rmagine::MemoryView<rmagine::Transform, rmagine::VRAM_CUDA> particle_poses,
  const rmagine::MemoryView<ParticleAttributes, rmagine::VRAM_CUDA> particle_attrs,
  const rmagine::MemoryView<SimpleLikelihoodStats, rmagine::VRAM_CUDA> stats,
  rmagine::MemoryView<curandState, rmagine::VRAM_CUDA> rstates,
  rmagine::MemoryView<rmagine::Transform, rmagine::VRAM_CUDA> particle_poses_new,
  rmagine::MemoryView<ParticleAttributes, rmagine::VRAM_CUDA> particle_attrs_new,
  const ParticleUpdateDynamicConfig& config)
{
  const unsigned int n_particles = particle_poses.size();
  constexpr unsigned int blockSize = 1024;
  const unsigned int gridSize = (particle_poses.size() + blockSize - 1) / blockSize;

  tournament_resample_kernel<<<gridSize, blockSize>>>(
    particle_poses.raw(), particle_attrs.raw(), stats.raw(), rstates.raw(),
    particle_poses_new.raw(), particle_attrs_new.raw(), n_particles, config);
}




} // namespace rmcl