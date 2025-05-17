#ifndef RMCL_ROS_CORRECTION_RCC_EMBREE_HPP
#define RMCL_ROS_CORRECTION_RCC_EMBREE_HPP

#include <rmagine/types/Memory.hpp>
#include <rmagine/types/PointCloud.hpp>
#include <rmagine/map/EmbreeMap.hpp>

#include <rmagine/simulation/SphereSimulatorEmbree.hpp>
#include <rmagine/simulation/PinholeSimulatorEmbree.hpp>
#include <rmagine/simulation/O1DnSimulatorEmbree.hpp>
#include <rmagine/simulation/OnDnSimulatorEmbree.hpp>

#include <rmcl/registration/CorrespondencesCPU.hpp>

namespace rmcl
{

class RCCEmbreeSpherical
: public CorrespondencesCPU
, public rmagine::ModelSetter<rmagine::SphericalModel>
, protected rmagine::SphereSimulatorEmbree
{
public:

  RCCEmbreeSpherical(
    rmagine::EmbreeMapPtr map);

  void setModel(const rmagine::SphericalModel& sensor_model);

  virtual void setTsb(const rmagine::Transform& Tsb) override;

  virtual void find(const rmagine::Transform& Tbm_est);
};

class RCCEmbreePinhole
: public CorrespondencesCPU
, public rmagine::ModelSetter<rmagine::PinholeModel>
, protected rmagine::PinholeSimulatorEmbree
{
public:
  RCCEmbreePinhole(
    rmagine::EmbreeMapPtr map);

  void setModel(const rmagine::PinholeModel& sensor_model);

  virtual void setTsb(const rmagine::Transform& Tsb) override;

  virtual void find(const rmagine::Transform& Tbm_est);
};

class RCCEmbreeO1Dn
: public CorrespondencesCPU
, public rmagine::ModelSetter<rmagine::O1DnModel>
, protected rmagine::O1DnSimulatorEmbree
{
public:

  RCCEmbreeO1Dn(
    rmagine::EmbreeMapPtr map);

  void setModel(const rmagine::O1DnModel& sensor_model);

  virtual void setTsb(const rmagine::Transform& Tsb) override;

  virtual void find(const rmagine::Transform& Tbm_est);
};

class RCCEmbreeOnDn
: public CorrespondencesCPU
, public rmagine::ModelSetter<rmagine::OnDnModel>
, protected rmagine::OnDnSimulatorEmbree
{
public:

  RCCEmbreeOnDn(
    rmagine::EmbreeMapPtr map);

  void setModel(const rmagine::OnDnModel& sensor_model);

  virtual void setTsb(const rmagine::Transform& Tsb) override;

  virtual void find(const rmagine::Transform& Tbm_est);
};

} // namespace rmcl

#endif // RMCL_ROS_CORRECTION_RCC_EMBREE_HPP