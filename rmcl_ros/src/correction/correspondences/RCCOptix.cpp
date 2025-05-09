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
  Correspondences_<rm::VRAM_CUDA>::setTsb(Tsb);
  rm::O1DnSimulatorOptix::setTsb(Tsb);
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

rm::PointCloudView_<rm::VRAM_CUDA> RCCOptixO1Dn::get()
{
  const rm::PointCloudView_<rm::VRAM_CUDA> cloud_model = {
    .points  = model_buffers_.points,
    .mask    = model_buffers_.hits,
    .normals = model_buffers_.normals
  };

  return cloud_model;
}

rmagine::CrossStatistics RCCEmbreeO1Dn::computeCrossStatistics(
  const rmagine::Transform& T_snew_sold) const
{
  std::cout << "computeCrossStatiscsGPU" << std::endl;
  // TODO: move this outside
  // params.max_dist = 1.0;

  const rm::PointCloudView_<rm::VRAM_CUDA> cloud_dataset = rm::watch(dataset);
  const rm::PointCloudView_<rm::VRAM_CUDA> cloud_model = {
    .points  = model_buffers_.points,
    .mask    = model_buffers_.hits,
    .normals = model_buffers_.normals
  };

  const rm::CrossStatistics stats_s = rm::statistics_p2l(T_snew_sold, cloud_dataset, cloud_model, params);
  return stats_s;
}

} // namespace rmcl