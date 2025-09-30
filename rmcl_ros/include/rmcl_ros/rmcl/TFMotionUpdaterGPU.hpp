#ifndef RMCL_MCL_TF_MOTION_UPDATER_GPU_HPP
#define RMCL_MCL_TF_MOTION_UPDATER_GPU_HPP

#include <rmagine/math/types.h>
#include <rmagine/types/MemoryCuda.hpp>

#include "ParticleAttributes.hpp"
#include "TFMotionUpdater.hpp"

#include <rmagine/map/MapMap.hpp>

#include <rmagine/map/OptixMap.hpp>

namespace rmcl
{

/**
 * @brief A 
 */
class TFMotionUpdaterGPU 
: public TFMotionUpdater<rmagine::VRAM_CUDA>
{
public:
  using Base = TFMotionUpdater<rmagine::VRAM_CUDA>;

  // inherit constructor
  using Base::Base;

  void init() override;

  ParticleUpdateResults update(
    rmagine::MemoryView<rmagine::Transform, rmagine::VRAM_CUDA> particle_poses,
    rmagine::MemoryView<ParticleAttributes, rmagine::VRAM_CUDA> particle_attrs,
    const ParticleUpdateConfig& config) override;

private:
  void updateParams();

  void loadMap();

  rmagine::OptixMapPtr map_;
};

} // namespace rmcl

#endif // RMCL_MCL_TF_MOTION_UPDATER_GPU_HPP