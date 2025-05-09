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
#include <rmcl_ros/correction/correspondences/CorrespondencesCPU.hpp>

#include <rmcl_ros/correction/sensors/ModelSetter.hpp>

namespace rmcl
{

class RCCEmbreeO1Dn
: public CorrespondencesCPU
, public ModelSetter<rmagine::O1DnModel>
, protected rmagine::O1DnSimulatorEmbree
{
public:

  RCCEmbreeO1Dn(
    rmagine::EmbreeMapPtr map);

  void setModel(const rmagine::O1DnModel& sensor_model);

  virtual void setTsb(const rmagine::Transform& Tsb) override;

  virtual void find(const rmagine::Transform& Tbm_est);
};

} // namespace rmcl

#endif // RMCL_ROS_CORRECTION_RCC_EMBREE_HPP