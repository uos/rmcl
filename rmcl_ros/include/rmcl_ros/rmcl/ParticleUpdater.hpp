#ifndef RMCL_PARTICLE_UPDATER_HPP
#define RMCL_PARTICLE_UPDATER_HPP

#include <rmagine/math/types.h>
#include <rmagine/types/Memory.hpp>
#include "ParticleAttributes.hpp"

namespace rmcl
{

struct ParticleUpdateResults
{
  // uint32_t n_particles;
};

struct ParticleUpdateConfig
{
  // uint32_t n_usable_particles;
};

/**
 * @brief base class for all implementations that update particles without resizing the number of elements
 */
template<typename MemT>
class ParticleUpdater
{
public:
  virtual ~ParticleUpdater() = default;

  /**
   * @brief Update a set of particles.
   * usable memory can be accessed via 
   * 
   * @code
   * auto particle_poses_usable = particle_poses(0, config.n_usable_particles);
   * 
   * @endcode
   */
  virtual ParticleUpdateResults update(
      rmagine::MemoryView<rmagine::Transform, MemT> particle_poses,
      rmagine::MemoryView<ParticleAttributes, MemT> particle_attrs,
      const ParticleUpdateConfig& config = {}
  ) = 0;
};




struct ParticleUpdateDynamicResults
{
  uint32_t n_particles;
};

struct ParticleUpdateDynamicConfig
{
  // uint32_t n_usable_particles;
};

template<typename MemT>
class ParticleUpdaterDynamic
{
public:
  virtual ~ParticleUpdaterDynamic() = default;


  virtual ParticleUpdateDynamicResults update(
      const rmagine::MemoryView<rmagine::Transform, MemT> particle_poses,
      const rmagine::MemoryView<ParticleAttributes, MemT> particle_attrs,
      rmagine::MemoryView<rmagine::Transform, MemT> particle_poses_new,
      rmagine::MemoryView<ParticleAttributes, MemT> particle_attrs_new,
      const ParticleUpdateDynamicConfig& config = {}
  ) = 0;
};


} // namespace rmcl

#endif // RMCL_PARTICLE_UPDATER_HPP