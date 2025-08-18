#include <rmcl_ros/rmcl/KLDResamplerCPU.hpp>
#include <random>
#include <memory>

namespace rmcl
{

KLDResamplerCPU::KLDResamplerCPU()
{
  std::random_device device{};
  rand_gen_ = std::make_unique<std::mt19937>(device());
}

void KLDResamplerCPU::init()
{

}

void KLDResamplerCPU::reset()
{

}

ParticleUpdateDynamicResults KLDResamplerCPU::update(
  const rmagine::MemoryView<rmagine::Transform, rmagine::RAM> particle_poses,
  const rmagine::MemoryView<ParticleAttributes, rmagine::RAM> particle_attrs,
  rmagine::MemoryView<rmagine::Transform, rmagine::RAM> particle_poses_new,
  rmagine::MemoryView<ParticleAttributes, rmagine::RAM> particle_attrs_new,
  const ParticleUpdateDynamicConfig& config)
{
  ParticleUpdateDynamicResults res;


  // this is a wheel resampler

  // res.n_particles = particle_poses.size();


  // particle_poses_new = particle_poses;

  // float weight_sum = 0.0;
  // for(size_t i=0; i<particle_poses.size(); i++)
  // {
  //   weight_sum += particle_attrs[i].likelihood.mean;
  // }

  // static std::uniform_real_distribution<float> uni_dist(0.0, 1.0);

  // // worst case O(n^2) runtime. this is not good
  // // sorting only for a single operation would also not be good
  // for (auto particle_index = 0u; particle_index < particle_poses.size(); ++particle_index)
  // {
  //     float random_value = uni_dist(*rand_gen_);
  //     float weight_sum = 0.0;

  //     for(auto index = 0u; index < particle_poses.size(); index++)
  //     { 
  //         float current_weight = particle_attrs[index].likelihood.mean;
  //         weight_sum += current_weight;

  //         if(random_value <= weight_sum)
  //         {
  //             particle_poses_new[particle_index] = particle_poses[index];
  //             particle_attrs_new[particle_index] = particle_attrs[index];
  //             break;
  //         }
  //     }

  //     //particle_cloud[particle_index] = tmp_particle_cloud[particle_cloud.size() - 1];
  // }

  throw std::runtime_error("not implemented");

  return res;
}

} // namespace rmcl