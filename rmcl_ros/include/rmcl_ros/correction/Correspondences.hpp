#ifndef RMCL_CORRECTION_CORRESPONDENCES_HPP
#define RMCL_CORRECTION_CORRESPONDENCES_HPP

#include <rmagine/math/types.h>
#include <rmagine/types/PointCloud.hpp>
#include <rmagine/types/Bundle.hpp>
#include <rmagine/types/Memory.hpp>
#include <rmagine/simulation/SimulationResults.hpp>

#include <rmagine/math/statistics.h>



namespace rmcl
{

template<typename MemT>
class Correspondences_
{
public:

  rmagine::UmeyamaReductionConstraints params;

  // fill this
  rmagine::PointCloud_<MemT> dataset;

  virtual void setTsb(const rmagine::Transform& Tsb)
  {
    Tsb_ = Tsb;
  };

  /**
   * Finds and fill model buffers
   */
  virtual void find(
    const rmagine::Transform& Tbm_est
  ) = 0;

  /**
   * Get model buffers as rmagine pointcloud
   */
  virtual rmagine::PointCloudView_<MemT> get() = 0;

  bool outdated = true;

  virtual rmagine::CrossStatistics computeCrossStatistics( 
    const rmagine::Transform& T_snew_sold) const = 0;

protected:

  rmagine::Bundle<
    rmagine::Points<MemT>,  // model points
    rmagine::Normals<MemT>, // model normals
    rmagine::Hits<MemT>     // correspondence mask
    > model_buffers_;

  rmagine::Transform Tsb_;
};

} // namespace rmcl

#endif // RMCL_CORRECTION_CORRESPONDENCES_HPP