#ifndef RMCL_GLADIATOR_RESAMPLER_CONFIG_HPP
#define RMCL_GLADIATOR_RESAMPLER_CONFIG_HPP

namespace rmcl
{

struct GladiatorResamplerConfig {
  
  float min_noise_tx; // motion + sensor noise
  float min_noise_ty;
  float min_noise_tz;
  float min_noise_roll;
  float min_noise_pitch;
  float min_noise_yaw;

  // dependent how far from the original sample 
  float likelihood_forget_per_meter = 0.3;
  float likelihood_forget_per_radian = 0.2;

};

} // namespace rmcl

#endif // RMCL_GLADIATOR_RESAMPLER_CONFIG_HPP