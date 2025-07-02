#include "rmcl/registration/RCCOptix.hpp"

namespace rm = rmagine;

namespace rmcl
{


RCCOptixSpherical::RCCOptixSpherical(
  rm::OptixMapPtr map)
: rm::SphereSimulatorOptix(map)
{
  
}

void RCCOptixSpherical::setTsb(const rm::Transform& Tsb)
{
  CorrespondencesCUDA::setTsb(Tsb);
  rm::SphereSimulatorOptix::setTsb(Tsb);
}

void RCCOptixSpherical::setModel(const rm::SphericalModel& sensor_model)
{
  rm::SphereSimulatorOptix::setModel(sensor_model);
  model_cache_ = sensor_model;
}

void RCCOptixSpherical::find(const rm::Transform& Tbm_est)
{
  const size_t n_new_measurements = model_cache_.size();
  if(n_new_measurements == 0)
  {
    return;
  }
  const size_t n_old_measurements = model_buffers_.points.size();
  if(n_new_measurements > n_old_measurements)
  {
    rm::resize_memory_bundle<rm::VRAM_CUDA>(model_buffers_, 
      model_cache_.getHeight(), model_cache_.getWidth(), 1);
  }
  
  simulate(Tbm_est, model_buffers_);
}


RCCOptixPinhole::RCCOptixPinhole(
  rm::OptixMapPtr map)
: rm::PinholeSimulatorOptix(map)
{
  
}

void RCCOptixPinhole::setTsb(const rm::Transform& Tsb)
{
  CorrespondencesCUDA::setTsb(Tsb);
  rm::PinholeSimulatorOptix::setTsb(Tsb);
}

void RCCOptixPinhole::setModel(const rm::PinholeModel& sensor_model)
{
  rm::PinholeSimulatorOptix::setModel(sensor_model);
  model_cache_ = sensor_model;
}

void RCCOptixPinhole::find(const rm::Transform& Tbm_est)
{
  size_t n_new_measurements = model_cache_.size();
  if(n_new_measurements == 0)
  {
    return;
  }

  size_t n_old_measurements = model_buffers_.points.size();
  if(n_new_measurements > n_old_measurements)
  {
    rm::resize_memory_bundle<rm::VRAM_CUDA>(model_buffers_, 
      model_cache_.getHeight(), model_cache_.getWidth(), 1);
  }

  simulate(Tbm_est, model_buffers_);
}


RCCOptixO1Dn::RCCOptixO1Dn(
  rm::OptixMapPtr map)
: rm::O1DnSimulatorOptix(map)
{
  
}

void RCCOptixO1Dn::setTsb(const rm::Transform& Tsb)
{
  CorrespondencesCUDA::setTsb(Tsb);
  rm::O1DnSimulatorOptix::setTsb(Tsb);
}

void RCCOptixO1Dn::setModel(const rm::O1DnModel& sensor_model)
{
  rm::O1DnSimulatorOptix::setModel(sensor_model);
}

void RCCOptixO1Dn::find(const rm::Transform& Tbm_est)
{
  size_t n_new_measurements = m_model->size();
  if(n_new_measurements == 0)
  {
    return;
  }

  size_t n_old_measurements = model_buffers_.points.size();
  if(n_new_measurements > n_old_measurements)
  {
    rm::resize_memory_bundle<rm::VRAM_CUDA>(model_buffers_, 
      m_model->getHeight(), m_model->getWidth(), 1);
  }

  simulate(Tbm_est, model_buffers_);
}


RCCOptixOnDn::RCCOptixOnDn(
  rm::OptixMapPtr map)
: rm::OnDnSimulatorOptix(map)
{
  
}

void RCCOptixOnDn::setTsb(const rm::Transform& Tsb)
{
  CorrespondencesCUDA::setTsb(Tsb);
  rm::OnDnSimulatorOptix::setTsb(Tsb);
}

void RCCOptixOnDn::setModel(const rm::OnDnModel& sensor_model)
{
  rm::OnDnSimulatorOptix::setModel(sensor_model);
}

void RCCOptixOnDn::find(const rm::Transform& Tbm_est)
{
  size_t n_new_measurements = m_model->size();
  if(n_new_measurements == 0)
  {
    return;
  }
  
  size_t n_old_measurements = model_buffers_.points.size();
  if(n_new_measurements > n_old_measurements)
  {
    rm::resize_memory_bundle<rm::VRAM_CUDA>(model_buffers_, 
      m_model->getHeight(), m_model->getWidth(), 1);
  }

  simulate(Tbm_est, model_buffers_);
}


} // namespace rmcl