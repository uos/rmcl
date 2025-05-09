#include <rmcl_ros/correction/correspondences/CorrespondencesCPU.hpp>
#include <rmagine/math/statistics.h>

namespace rm = rmagine;

namespace rmcl
{

rmagine::CrossStatistics CorrespondencesCPU::computeCrossStatistics(
  const rmagine::Transform& T_snew_sold) const
{
  const rm::PointCloudView_<rm::RAM> cloud_dataset = rm::watch(dataset);
  const rm::PointCloudView_<rm::RAM> cloud_model = {
    .points  = model_buffers_.points,
    .mask    = model_buffers_.hits,
    .normals = model_buffers_.normals
  };

  const rm::CrossStatistics stats_s = rm::statistics_p2l(T_snew_sold, cloud_dataset, cloud_model, params);
  return stats_s;
}

} // namespace rmcl