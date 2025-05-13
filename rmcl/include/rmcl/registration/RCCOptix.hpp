#ifndef RMCL_ROS_CORRECTION_RCC_OPTIX_HPP
#define RMCL_ROS_CORRECTION_RCC_OPTIX_HPP

#include <rmagine/types/MemoryCuda.hpp>
#include <rmagine/types/PointCloud.hpp>
#include <rmagine/map/OptixMap.hpp>

#include <rmagine/simulation/SphereSimulatorOptix.hpp>
#include <rmagine/simulation/PinholeSimulatorOptix.hpp>
#include <rmagine/simulation/O1DnSimulatorOptix.hpp>
#include <rmagine/simulation/OnDnSimulatorOptix.hpp>

#include <rmcl/registration/CorrespondencesCUDA.hpp>

namespace rmcl
{

class RCCOptixO1Dn
: public CorrespondencesCUDA
, public rmagine::ModelSetter<rmagine::O1DnModel>
, protected rmagine::O1DnSimulatorOptix
{
public:

  RCCOptixO1Dn(
    rmagine::OptixMapPtr map);

  void setModel(const rmagine::O1DnModel& sensor_model);

  virtual void setTsb(const rmagine::Transform& Tsb) override;

  virtual void find(const rmagine::Transform& Tbm_est);
};

} // namespace rmcl

#endif // RMCL_ROS_CORRECTION_RCC_EMBREE_HPP