#ifndef RMCL_CORRECTION_CORRESPONDENCES_HPP
#define RMCL_CORRECTION_CORRESPONDENCES_HPP

#include <rmagine/types/PointCloud.hpp>
#include <rmagine/types/Bundle.hpp>

namespace rmcl
{

template<typename MemT>
class Correspondences_
{
public:

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

protected:

  rmagine::Bundle<
    rmagine::Points<rmagine::RAM>,  // model points
    rmagine::Normals<rmagine::RAM>, // model normals
    rmagine::Hits<rmagine::RAM>     // correspondence mask
    > model_buffers_;

  rmagine::Transform Tsb_;

};

} // namespace rmcl

#endif // RMCL_CORRECTION_CORRESPONDENCES_HPP