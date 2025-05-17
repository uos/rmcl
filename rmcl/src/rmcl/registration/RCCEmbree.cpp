#include "rmcl/registration/RCCEmbree.hpp"

namespace rm = rmagine;

namespace rmcl
{

RCCEmbreeSpherical::RCCEmbreeSpherical(
  rm::EmbreeMapPtr map)
: rm::SphereSimulatorEmbree(map)
{
  
}

void RCCEmbreeSpherical::setTsb(const rm::Transform& Tsb)
{
  CorrespondencesCPU::setTsb(Tsb);
  rm::SphereSimulatorEmbree::setTsb(Tsb);
}

void RCCEmbreeSpherical::setModel(const rm::SphericalModel& sensor_model)
{
  rm::SphereSimulatorEmbree::setModel(sensor_model);
}

void RCCEmbreeSpherical::find(const rm::Transform& Tbm_est)
{
  size_t n_old_measurements = model_buffers_.points.size();
  size_t n_new_measurements = m_model->size();
  if(n_new_measurements > n_old_measurements)
  {
    rm::resize_memory_bundle<rm::RAM>(model_buffers_, m_model->getHeight(), m_model->getWidth(), 1);
  }

  simulate(Tbm_est, model_buffers_);
}



RCCEmbreePinhole::RCCEmbreePinhole(
  rm::EmbreeMapPtr map)
: rm::PinholeSimulatorEmbree(map)
{
  
}

void RCCEmbreePinhole::setTsb(const rm::Transform& Tsb)
{
  CorrespondencesCPU::setTsb(Tsb);
  rm::PinholeSimulatorEmbree::setTsb(Tsb);
}

void RCCEmbreePinhole::setModel(const rm::PinholeModel& sensor_model)
{
  rm::PinholeSimulatorEmbree::setModel(sensor_model);
}

void RCCEmbreePinhole::find(const rm::Transform& Tbm_est)
{
  size_t n_old_measurements = model_buffers_.points.size();
  size_t n_new_measurements = m_model->size();
  if(n_new_measurements > n_old_measurements)
  {
    rm::resize_memory_bundle<rm::RAM>(model_buffers_, m_model->getHeight(), m_model->getWidth(), 1);
  }

  simulate(Tbm_est, model_buffers_);
}


RCCEmbreeO1Dn::RCCEmbreeO1Dn(
  rm::EmbreeMapPtr map)
: rm::O1DnSimulatorEmbree(map)
{
  
}

void RCCEmbreeO1Dn::setTsb(const rm::Transform& Tsb)
{
  CorrespondencesCPU::setTsb(Tsb);
  rm::O1DnSimulatorEmbree::setTsb(Tsb);
}

void RCCEmbreeO1Dn::setModel(const rm::O1DnModel& sensor_model)
{
  rm::O1DnSimulatorEmbree::setModel(sensor_model);
}

void RCCEmbreeO1Dn::find(const rm::Transform& Tbm_est)
{
  size_t n_old_measurements = model_buffers_.points.size();
  size_t n_new_measurements = m_model->size();
  if(n_new_measurements > n_old_measurements)
  {
    rm::resize_memory_bundle<rm::RAM>(model_buffers_, m_model->getHeight(), m_model->getWidth(), 1);
  }

  simulate(Tbm_est, model_buffers_);
}



RCCEmbreeOnDn::RCCEmbreeOnDn(
  rm::EmbreeMapPtr map)
: rm::OnDnSimulatorEmbree(map)
{
  
}

void RCCEmbreeOnDn::setTsb(const rm::Transform& Tsb)
{
  CorrespondencesCPU::setTsb(Tsb);
  rm::OnDnSimulatorEmbree::setTsb(Tsb);
}

void RCCEmbreeOnDn::setModel(const rm::OnDnModel& sensor_model)
{
  rm::OnDnSimulatorEmbree::setModel(sensor_model);
}

void RCCEmbreeOnDn::find(const rm::Transform& Tbm_est)
{
  size_t n_old_measurements = model_buffers_.points.size();
  size_t n_new_measurements = m_model->size();
  if(n_new_measurements > n_old_measurements)
  {
    rm::resize_memory_bundle<rm::RAM>(model_buffers_, m_model->getHeight(), m_model->getWidth(), 1);
  }

  simulate(Tbm_est, model_buffers_);
}



} // namespace rmcl