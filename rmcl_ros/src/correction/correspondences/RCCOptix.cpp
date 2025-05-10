#include "rmcl_ros/correction/correspondences/RCCOptix.hpp"

namespace rm = rmagine;

namespace rmcl
{

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
  size_t n_old_measurements = model_buffers_.points.size();
  size_t n_new_measurements = m_model->size();
  if(n_new_measurements > n_old_measurements)
  {
    rm::resize_memory_bundle<rm::VRAM_CUDA>(model_buffers_, m_model->getHeight(), m_model->getWidth(), 1);
  }

  simulate(Tbm_est, model_buffers_);
}

} // namespace rmcl