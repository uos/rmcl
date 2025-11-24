#ifndef RMCL_ROS_CORRECTION_RCC_VULKAN_HPP
#define RMCL_ROS_CORRECTION_RCC_VULKAN_HPP

#include <rmagine/types/MemoryCuda.hpp>
#include <rmagine/types/PointCloud.hpp>
#include <rmagine/map/VulkanMap.hpp>

#include <rmagine/simulation/SphereSimulatorVulkan.hpp>
#include <rmagine/simulation/PinholeSimulatorVulkan.hpp>
#include <rmagine/simulation/O1DnSimulatorVulkan.hpp>
#include <rmagine/simulation/OnDnSimulatorVulkan.hpp>

#include <rmagine/types/MemoryVulkanCudaInterop.hpp>

#include <rmcl/registration/CorrespondencesCUDA.hpp>

namespace rm = rmagine;

namespace rmcl
{

class RCCVulkanSpherical
: public CorrespondencesCUDA
, public rmagine::ModelSetter<rmagine::SphericalModel>
, protected rmagine::SphereSimulatorVulkan
{
public:

  RCCVulkanSpherical(
    rmagine::VulkanMapPtr map);

  void setModel(const rmagine::SphericalModel& sensor_model);

  virtual void setTsb(const rmagine::Transform& Tsb) override;

  virtual void find(const rmagine::Transform& Tbm_est);

private:

  rmagine::SphericalModel model_cache_;

  //TODO: mapping in gegenrichtung nutzen, sobald es in rmagine existiert
  rmagine::Bundle<
    rmagine::Points<rm::DEVICE_LOCAL_VULKAN>,  // model points
    rmagine::Normals<rm::DEVICE_LOCAL_VULKAN>, // model normals
    rmagine::Hits<rm::DEVICE_LOCAL_VULKAN>     // correspondence mask
    > model_buffers_vulkan_;
};


class RCCVulkanPinhole
: public CorrespondencesCUDA
, public rmagine::ModelSetter<rmagine::PinholeModel>
, protected rmagine::PinholeSimulatorVulkan
{
public:

  RCCVulkanPinhole(
    rmagine::VulkanMapPtr map);

  void setModel(const rmagine::PinholeModel& sensor_model);

  virtual void setTsb(const rmagine::Transform& Tsb) override;

  virtual void find(const rmagine::Transform& Tbm_est);

private:
  rmagine::PinholeModel model_cache_;

  //TODO: mapping in gegenrichtung nutzen, sobald es in rmagine existiert
  rmagine::Bundle<
    rmagine::Points<rm::DEVICE_LOCAL_VULKAN>,  // model points
    rmagine::Normals<rm::DEVICE_LOCAL_VULKAN>, // model normals
    rmagine::Hits<rm::DEVICE_LOCAL_VULKAN>     // correspondence mask
    > model_buffers_vulkan_;
};


class RCCVulkanO1Dn
: public CorrespondencesCUDA
, public rmagine::ModelSetter<rmagine::O1DnModel>
, protected rmagine::O1DnSimulatorVulkan
{
public:

  RCCVulkanO1Dn(
    rmagine::VulkanMapPtr map);

  void setModel(const rmagine::O1DnModel& sensor_model);

  virtual void setTsb(const rmagine::Transform& Tsb) override;

  virtual void find(const rmagine::Transform& Tbm_est);

private:
  rmagine::O1DnModel model_cache_;

  //TODO: mapping in gegenrichtung nutzen, sobald es in rmagine existiert
  rmagine::Bundle<
    rmagine::Points<rm::DEVICE_LOCAL_VULKAN>,  // model points
    rmagine::Normals<rm::DEVICE_LOCAL_VULKAN>, // model normals
    rmagine::Hits<rm::DEVICE_LOCAL_VULKAN>     // correspondence mask
    > model_buffers_vulkan_;
};

class RCCVulkanOnDn
: public CorrespondencesCUDA
, public rmagine::ModelSetter<rmagine::OnDnModel>
, protected rmagine::OnDnSimulatorVulkan
{
public:

  RCCVulkanOnDn(
    rmagine::VulkanMapPtr map);

  void setModel(const rmagine::OnDnModel& sensor_model);

  virtual void setTsb(const rmagine::Transform& Tsb) override;

  virtual void find(const rmagine::Transform& Tbm_est);

private:
  rmagine::OnDnModel model_cache_;

  //TODO: mapping in gegenrichtung nutzen, sobald es in rmagine existiert
  rmagine::Bundle<
    rmagine::Points<rm::DEVICE_LOCAL_VULKAN>,  // model points
    rmagine::Normals<rm::DEVICE_LOCAL_VULKAN>, // model normals
    rmagine::Hits<rm::DEVICE_LOCAL_VULKAN>     // correspondence mask
    > model_buffers_vulkan_;
};


} // namespace rmcl

#endif // RMCL_ROS_CORRECTION_RCC_EMBREE_HPP