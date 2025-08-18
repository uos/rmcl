#ifndef RMCL_MCL_RESAMPLER_HPP
#define RMCL_MCL_RESAMPLER_HPP

#include "ParticleUpdater.hpp"
#include <memory>

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