#ifndef RMCL_MCL_RESAMPLER_HPP
#define RMCL_MCL_RESAMPLER_HPP

#include "ParticleUpdater.hpp"
#include <memory>
#include <rmagine/math/types.h>

#include <deque>

#include <rmcl_msgs/msg/particle_stats.hpp>

namespace rmcl
{

class ResamplerBase
{
public:
  virtual ~ResamplerBase() = default;

  /**
   * @brief things that need to be done once at the beginning
   */
  virtual void init() = 0;

  virtual void reset() = 0;

  inline void addStats(rmcl_msgs::msg::ParticleStats stats) {
    const size_t history_size = 10;
    if (stats_.size() == history_size) 
    {
      stats_.pop_front();
    }
    stats_.push_back(stats);
  }

protected:

  std::deque<rmcl_msgs::msg::ParticleStats> stats_;
};

template<typename MemT>
class Resampler
: public ResamplerBase
, public ParticleUpdaterDynamic<MemT>
{
};

template<typename MemT>
using ResamplerPtr = std::shared_ptr<Resampler<MemT> >;

} // namespace rmcl

#endif // RMCL_MCL_RESAMPLER_HPP