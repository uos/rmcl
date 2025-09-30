#ifndef RMCL_MCL_TF_MOTION_UPDATER_CPU_HPP
#define RMCL_MCL_TF_MOTION_UPDATER_CPU_HPP

#include <rmagine/math/types.h>
#include <rmagine/types/Memory.hpp>

#include "ParticleAttributes.hpp"
#include "TFMotionUpdater.hpp"

#include <rmagine/map/MapMap.hpp>

#include <rmagine/map/EmbreeMap.hpp>

namespace rmcl
{

/**
 * @brief A 
 */
class TFMotionUpdaterCPU 
: public TFMotionUpdater<rmagine::RAM>
{
public:
  using Base = TFMotionUpdater<rmagine::RAM>;

  // inherit constructor
  using Base::Base;

  void init() override;

  ParticleUpdateResults update(
    rmagine::MemoryView<rmagine::Transform, rmagine::RAM> particle_poses,
    rmagine::MemoryView<ParticleAttributes, rmagine::RAM> particle_attrs,
    const ParticleUpdateConfig& config) override;

private:
  void updateParams();

  void loadMap();

  rmagine::EmbreeMapPtr map_;
};

} // namespace rmcl

#endif // RMCL_MCL_TF_MOTION_UPDATER_CPU_HPP