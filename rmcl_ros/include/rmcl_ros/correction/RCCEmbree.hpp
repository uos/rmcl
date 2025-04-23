#ifndef RMCL_ROS_CORRECTION_RCC_EMBREE_HPP
#define RMCL_ROS_CORRECTION_RCC_EMBREE_HPP

#include <rmagine/types/Memory.hpp>
#include <rmagine/types/PointCloud.hpp>
#include <rmagine/map/EmbreeMap.hpp>

#include <rmagine/simulation/SphereSimulatorEmbree.hpp>
#include <rmagine/simulation/PinholeSimulatorEmbree.hpp>
#include <rmagine/simulation/O1DnSimulatorEmbree.hpp>
#include <rmagine/simulation/OnDnSimulatorEmbree.hpp>

#include <rmcl_ros/correction/CorrespondenceFinder.hpp>

namespace rmcl
{


class RCCEmbreeSpherical
: public CorrespondenceFinder_<rmagine::RAM>
, public rmagine::SphereSimulatorEmbree
{
public:
  RCCEmbreeSpherical(rmagine::EmbreeMapPtr map)
  : CorrespondenceFinder_<rmagine::RAM>()
  , rmagine::SphereSimulatorEmbree(map)
  {
    
  }
};

class RCCEmbreeO1Dn
: public CorrespondenceFinder_<rmagine::RAM>
, public rmagine::O1DnSimulatorEmbree
{
public:
  RCCEmbreeO1Dn(rmagine::EmbreeMapPtr map)
  : CorrespondenceFinder_<rmagine::RAM>()
  , rmagine::O1DnSimulatorEmbree(map)
  {
    
  }
};

} // namespace rmcl

#endif // RMCL_ROS_CORRECTION_RCC_EMBREE_HPP