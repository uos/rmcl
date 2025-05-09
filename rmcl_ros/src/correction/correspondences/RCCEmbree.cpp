#include "rmcl_ros/correction/correspondences/RCCEmbree.hpp"

namespace rm = rmagine;

namespace rmcl
{

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

void RCCEmbreeO1Dn::setModel(const rmagine::O1DnModel& sensor_model)
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



} // namespace rmcl