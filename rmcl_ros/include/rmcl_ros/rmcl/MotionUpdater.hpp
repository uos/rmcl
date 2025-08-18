#ifndef RMCL_MCL_MOTION_UPDATER_HPP
#define RMCL_MCL_MOTION_UPDATER_HPP

#include <memory>

#include <rmagine/math/types.h>
#include <rmagine/types/Memory.hpp>

#include "ParticleAttributes.hpp"

#include "ParticleUpdater.hpp"


namespace rmcl
{

class MotionUpdaterBase
{
public:
  virtual ~MotionUpdaterBase() = default;

  /**
   * @brief things that need to be done once at the beginning
   */
  virtual void init() = 0;

  virtual void reset() = 0;
};

using MotionUpdaterBasePtr = std::shared_ptr<MotionUpdaterBase>;

/**
 * @brief A MotionUpdate computes prior states. moves
 * 
 * This is a pure virtual class which you can implement for a specific
 * - map representation (collisions)
 * - likelihood degrading by collisions
 * 
 */
template<typename MemT>
class MotionUpdater 
: public MotionUpdaterBase
, public ParticleUpdater<MemT>
{
  
};

template<typename MemT>
using MotionUpdaterPtr = std::shared_ptr<MotionUpdater<MemT> >;

} // namespace rmcl

#endif // RMCL_MCL_MOTION_UPDATER_HPP