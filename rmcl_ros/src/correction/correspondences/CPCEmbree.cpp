#include "rmcl_ros/correction/correspondences/CPCEmbree.hpp"

namespace rm = rmagine;

namespace rmcl
{

CPCEmbree::CPCEmbree(
  rm::EmbreeMapPtr map)
:map_(map)
{
  
}

void CPCEmbree::find(const rm::Transform& Tbm_est)
{
  size_t n_old_measurements = model_buffers_.points.size();
  size_t n_new_measurements = dataset.points.size();
  if(n_new_measurements > n_old_measurements)
  {
    rm::resize_memory_bundle<rm::RAM>(model_buffers_, dataset.points.size(), 1, 1);
  }

  const rm::Transform Tsm = Tbm_est * Tsb_;
  const rm::Transform Tms = ~Tsm;

  #pragma omp parallel for
  for(size_t i=0; i<dataset.points.size(); i++)
  {
    const rm::Point Pm = Tsm * dataset.points[i];
    const rm::EmbreeClosestPointResult cp = map_->closestPoint(Pm);

    model_buffers_.hits[i] = (cp.d <= params.max_dist);
    model_buffers_.points[i] = Tms * cp.p;
    model_buffers_.normals[i] = Tms.R * cp.n;
  }
}


} // namespace rmcl