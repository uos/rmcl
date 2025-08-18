#ifndef RMCL_MCL_SENSOR_UPDATER_HPP
#define RMCL_MCL_SENSOR_UPDATER_HPP

#include <memory>

#include <rmagine/math/types.h>
#include <rmagine/types/Memory.hpp>

#include "ParticleAttributes.hpp"

#include "ParticleUpdater.hpp"


namespace rmcl
{


class SensorUpdaterBase
{
public:
  virtual ~SensorUpdaterBase() = default;

  virtual void init() = 0;
  virtual void reset() = 0;
};

using SensorUpdaterBasePtr = std::shared_ptr<SensorUpdaterBase>;

/**
 * @brief A SensorUpdate computes posterior likelihoods
 * 
 * This is a pure virtual class which you can implement for a specific
 * - map representation
 * - likelihood computation
 * 
 */
template<typename MemT>
class SensorUpdater 
: public SensorUpdaterBase
, public ParticleUpdater<MemT>
{
};

template<typename MemT>
using SensorUpdaterPtr = std::shared_ptr<SensorUpdater<MemT> >;

} // namespace rmcl

#endif // RMCL_MCL_SENSOR_UPDATER_HPP