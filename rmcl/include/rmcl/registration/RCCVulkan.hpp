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

/**
 * this is not a very mice way to handle the views, but i could not find a better way right now...
 * 
 * views need to be recreated every time the results buffers get resized...
 * 
 * however using these views to copy from vulkan memory to cuda memory is faster than copying directly, 
 * because you dont need to fetch a file descriptor every time.
 */
struct ViewContainer
{
  ViewContainer(rm::MemoryView<uint8_t, rm::DEVICE_LOCAL_VULKAN>& hits, 
                rm::MemoryView<rm::Point, rm::DEVICE_LOCAL_VULKAN>& points, 
                rm::MemoryView<rm::Vector, rm::DEVICE_LOCAL_VULKAN>& normals) :
                hits_view(hits), points_view(points), normals_view(normals) {}

  rmagine::MemoryView<uint8_t, rm::VULKAN_AS_CUDA> hits_view;
  rmagine::MemoryView<rm::Point, rm::VULKAN_AS_CUDA> points_view;
  rmagine::MemoryView<rm::Vector, rm::VULKAN_AS_CUDA> normals_view;
};
using ViewContainerPtr = std::unique_ptr<ViewContainer>;



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

  ViewContainerPtr viewContainer = nullptr;
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

  ViewContainerPtr viewContainer = nullptr;
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

  ViewContainerPtr viewContainer = nullptr;
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

  ViewContainerPtr viewContainer = nullptr;
};


} // namespace rmcl

#endif // RMCL_ROS_CORRECTION_RCC_EMBREE_HPP