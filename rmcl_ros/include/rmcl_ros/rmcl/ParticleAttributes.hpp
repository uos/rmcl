#ifndef RMCL_MCL_PARTICLE_ATTRIBUTES_HPP
#define RMCL_MCL_PARTICLE_ATTRIBUTES_HPP

#include <rmagine/math/types.h>

namespace rmcl
{


/**
 * This is collecting
 */
struct ParticleState
{
  rmagine::Transform pose;
};

struct ParticleAttributes
{
  // likelihood represented as Gaussian1D:
  // - incrementally collecting likelihood over time
  // - n_meas are representing the number of measurement taken to build the likelihood mean and sigma
  // n_meas role:
  // - as we collect data at a certain pose at a certain time n_meas will increase
  // - the more we move, the longer we wait, n_meas will decrease (parameters)
  //    - TODO: figure out how we can model classic MCL with this (im sure it's possible)
  rmagine::Gaussian1D likelihood;

  // this discribes the state uncertainty in odom coordinates
  // a priori
  rmagine::Matrix_<float, 6, 1> state_sigma;
};

constexpr static uint32_t MAX_N_MEAS = 10000;

} // namespace rmcl

#endif // RMCL_MCL_PARTICLE_ATTRIBUTES_HPP