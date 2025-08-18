#ifndef RMCL_MCL_KLD_RESAMPLER_CPU_HPP
#define RMCL_MCL_KLD_RESAMPLER_CPU_HPP

#include "Resampler.hpp"
#include "ParticleAttributes.hpp"

#include <random>
#include <memory>

namespace rmcl
{

class KLDResamplerCPU
: Resampler<rmagine::RAM>
{
public:
  KLDResamplerCPU();

  void reset() override;

  void init() override;

  ParticleUpdateDynamicResults update(
    const rmagine::MemoryView<rmagine::Transform, rmagine::RAM> particle_poses,
    const rmagine::MemoryView<ParticleAttributes, rmagine::RAM> particle_attrs,
    rmagine::MemoryView<rmagine::Transform, rmagine::RAM> particle_poses_new,
    rmagine::MemoryView<ParticleAttributes, rmagine::RAM> particle_attrs_new,
    const ParticleUpdateDynamicConfig& config) override;

  std::unique_ptr<std::mt19937> rand_gen_;
};

} // namespace rmcl

#endif // RMCL_MCL_KLD_RESAMPLER_CPU_HPP