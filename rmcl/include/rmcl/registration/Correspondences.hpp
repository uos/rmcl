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
  float adaptive_max_dist_min;

  // fill this
  rmagine::PointCloud_<MemT> dataset;

  virtual void setTsb(const rmagine::Transform& Tsb)
  {
    Tsb_ = Tsb;
  };

  /**
   * Finds and fill model buffers
   * 
   */
  virtual void find(
    const rmagine::Transform& Tbm_est 
  ) = 0;

  bool outdated = true;

  inline rmagine::PointCloudView_<MemT> modelView()
  {
    return rmagine::PointCloudView_<MemT>{
      .points = model_buffers_.points,
      .mask = model_buffers_.hits,
      .normals = model_buffers_.normals
    };
  }

  inline rmagine::PointCloudView_<MemT> datasetView()
  {
    return rmagine::PointCloudView_<MemT>{
      .points = dataset.points,
      .mask = dataset.mask
    };
  }

  /**
   * 
   * Computes cross statistics of correspondences that 
   * can be used to solve the registration problem 
   * via Kabsch/Umeyama.
   * 
   * @param convergence_progress value (in [0,1]) describes an 
   * estimate of convergence. (0: not converged at all, 1: fully 
   * converged). It can be used, eg, to adapt a maximum search 
   * distance. Otherwise this value can be ignored.
   */
  virtual rmagine::CrossStatistics computeCrossStatistics( 
    const rmagine::Transform& T_snew_sold,
    double convergence_progress = 0.0) const = 0;

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