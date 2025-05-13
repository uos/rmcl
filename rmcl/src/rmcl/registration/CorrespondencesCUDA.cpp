#include "rmcl/registration/CorrespondencesCUDA.hpp"
#include <rmagine/math/statistics.cuh>

namespace rm = rmagine;

namespace rmcl
{

rm::CrossStatistics CorrespondencesCUDA::computeCrossStatistics(
  const rm::Transform& T_snew_sold,
  double convergence_progress) const
{
  const rm::PointCloudView_<rm::VRAM_CUDA> cloud_dataset = rm::watch(dataset);
  const rm::PointCloudView_<rm::VRAM_CUDA> cloud_model = {
    .points  = model_buffers_.points,
    .mask    = model_buffers_.hits,
    .normals = model_buffers_.normals
  };

  // linearly interpolate between params.max_dist and adaptive_max_dist_min
  //   convergence_progress == 0.0 -> max_dist = params.max_dist
  //   convergence_progress == 1.0 -> max_dist = adaptive_max_dist_min
  rm::UmeyamaReductionConstraints params_local = params;
  params_local.max_dist = params.max_dist * (1.0 - convergence_progress) 
                          + adaptive_max_dist_min * convergence_progress;


  const rm::CrossStatistics stats_s = rm::statistics_p2l(T_snew_sold, cloud_dataset, cloud_model, params_local);
  return stats_s;
}

} // namespace rmcl