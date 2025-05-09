#ifndef RMCL_CORRECTION_CORRESPONDENCES_CPC_EMBREE_HPP
#define RMCL_CORRECTION_CORRESPONDENCES_CPC_EMBREE_HPP

#include <rmagine/math/types.h>
#include <rmagine/types/Memory.hpp>
#include <rmagine/types/PointCloud.hpp>
#include <rmagine/map/EmbreeMap.hpp>

#include <rmcl_ros/correction/Correspondences.hpp>
#include <rmcl_ros/correction/correspondences/CorrespondencesCPU.hpp>


namespace rmcl
{

class CPCEmbree
: public CorrespondencesCPU
{
public:
  CPCEmbree(rmagine::EmbreeMapPtr map);

  void find(const rmagine::Transform& Tbm_est);

protected:
  rmagine::EmbreeMapPtr map_;
};

} // namespace rmcl

#endif // RMCL_CORRECTION_CORRESPONDENCES_CPC_EMBREE_HPP