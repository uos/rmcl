#ifndef RMCL_ROS_CORRECTION_RCC_EMBREE_HPP
#define RMCL_ROS_CORRECTION_RCC_EMBREE_HPP

#include <rmagine/types/Memory.hpp>
#include <rmagine/types/PointCloud.hpp>
#include <rmagine/map/EmbreeMap.hpp>

#include <rmagine/simulation/SphereSimulatorEmbree.hpp>
#include <rmagine/simulation/PinholeSimulatorEmbree.hpp>
#include <rmagine/simulation/O1DnSimulatorEmbree.hpp>
#include <rmagine/simulation/OnDnSimulatorEmbree.hpp>

#include <rmcl_ros/correction/Correspondences.hpp>

namespace rmcl
{

// class CrossStatisticsComputerCPU
// {
//   virtual rmagine::CrossStatistics computeCrossStatistics(
//     const rmagine::Transform& T_bnew_bold // Tpre_b
//   ) const
// };



// class RCCEmbree
// : public Correspondences_<rmagine::RAM>


class RCCEmbreeO1Dn
: public Correspondences_<rmagine::RAM>
, public rmagine::O1DnSimulatorEmbree
{
public:

  RCCEmbreeO1Dn(
    rmagine::EmbreeMapPtr map);

  virtual void setTsb(const rmagine::Transform& Tsb) override;

  virtual void find(const rmagine::Transform& Tbm_est);

  virtual rmagine::PointCloudView_<rmagine::RAM> get();

  rmagine::CrossStatistics computeCrossStatistics(
    const rmagine::Transform& T_snew_sold) const;
};

} // namespace rmcl

#endif // RMCL_ROS_CORRECTION_RCC_EMBREE_HPP