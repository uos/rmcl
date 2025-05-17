#include "rmcl/registration/CorrespondencesCPU.hpp"
#include <rmagine/math/statistics.h>
#include <rmagine/util/prints.h>

namespace rm = rmagine;

namespace rmcl
{

rmagine::CrossStatistics CorrespondencesCPU::computeCrossStatistics(
  const rmagine::Transform& T_snew_sold,
  double convergence_progress) const
{
  const rm::PointCloudView_<rm::RAM> cloud_dataset = rm::watch(dataset);
  const rm::PointCloudView_<rm::RAM> cloud_model = {
    .points  = model_buffers_.points,
    .mask    = model_buffers_.hits,
    .normals = model_buffers_.normals
  };

  rm::UmeyamaReductionConstraints params_local = params;
  params_local.max_dist = params.max_dist * (1.0 - convergence_progress) 
                          + adaptive_max_dist_min * convergence_progress;


  const rm::CrossStatistics stats_s = rm::statistics_p2l(
    T_snew_sold, 
    cloud_dataset, 
    cloud_model, 
    params_local);

  // std::cout << "CrossStatistics:" << std::endl;
  // std::cout << stats_s.n_meas << std::endl;
  // std::cout << stats_s.dataset_mean << " -> " << stats_s.model_mean << std::endl;
  // std::cout << stats_s.covariance << std::endl;
  

  return stats_s;
}

} // namespace rmcl