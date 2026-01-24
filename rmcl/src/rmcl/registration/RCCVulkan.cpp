#include "rmcl/registration/RCCVulkan.hpp"

namespace rm = rmagine;

namespace rmcl
{


RCCVulkanSpherical::RCCVulkanSpherical(
  rm::VulkanMapPtr map)
: rm::SphereSimulatorVulkan(map)
{
  
}

void RCCVulkanSpherical::setTsb(const rm::Transform& Tsb)
{
  CorrespondencesCUDA::setTsb(Tsb);
  rm::SphereSimulatorVulkan::setTsb(Tsb);
}

void RCCVulkanSpherical::setModel(const rm::SphericalModel& sensor_model)
{
  rm::SphereSimulatorVulkan::setModel(sensor_model);
  model_cache_ = sensor_model;
}

void RCCVulkanSpherical::find(const rm::Transform& Tbm_est)
{
  const size_t n_new_measurements = model_cache_.size();
  if(n_new_measurements == 0)
  {
    return;
  }

  const size_t n_old_measurements = model_buffers_.points.size();
  if(n_new_measurements != n_old_measurements)
  {
    rm::resize_memory_bundle<rm::VRAM_CUDA>(model_buffers_, 
      model_cache_.getHeight(), model_cache_.getWidth(), 1);

    rm::resize_memory_bundle<rm::DEVICE_LOCAL_VULKAN>(model_buffers_vulkan_, 
      model_cache_.getHeight(), model_cache_.getWidth(), 1);
    
    viewContainer.reset();
    viewContainer = std::make_unique<ViewContainer>(model_buffers_vulkan_.hits, model_buffers_vulkan_.points, model_buffers_vulkan_.normals);
  }
  
  simulate(Tbm_est, model_buffers_vulkan_);

  // model_buffers_.hits = model_buffers_vulkan_.hits;
  // model_buffers_.points = model_buffers_vulkan_.points;
  // model_buffers_.normals = model_buffers_vulkan_.normals;

  model_buffers_.hits = viewContainer->hits_view;
  model_buffers_.points = viewContainer->points_view;
  model_buffers_.normals = viewContainer->normals_view;
}


RCCVulkanPinhole::RCCVulkanPinhole(
  rm::VulkanMapPtr map)
: rm::PinholeSimulatorVulkan(map)
{
  
}

void RCCVulkanPinhole::setTsb(const rm::Transform& Tsb)
{
  CorrespondencesCUDA::setTsb(Tsb);
  rm::PinholeSimulatorVulkan::setTsb(Tsb);
}

void RCCVulkanPinhole::setModel(const rm::PinholeModel& sensor_model)
{
  rm::PinholeSimulatorVulkan::setModel(sensor_model);
  model_cache_ = sensor_model;
}

void RCCVulkanPinhole::find(const rm::Transform& Tbm_est)
{
  size_t n_new_measurements = model_cache_.size();
  if(n_new_measurements == 0)
  {
    return;
  }

  size_t n_old_measurements = model_buffers_.points.size();
  if(n_new_measurements != n_old_measurements)
  {
    rm::resize_memory_bundle<rm::VRAM_CUDA>(model_buffers_, 
      model_cache_.getHeight(), model_cache_.getWidth(), 1);

    rm::resize_memory_bundle<rm::DEVICE_LOCAL_VULKAN>(model_buffers_vulkan_, 
      model_cache_.getHeight(), model_cache_.getWidth(), 1);
    
    viewContainer.reset();
    viewContainer = std::make_unique<ViewContainer>(model_buffers_vulkan_.hits, model_buffers_vulkan_.points, model_buffers_vulkan_.normals);
  }
  
  simulate(Tbm_est, model_buffers_vulkan_);

  // model_buffers_.hits = model_buffers_vulkan_.hits;
  // model_buffers_.points = model_buffers_vulkan_.points;
  // model_buffers_.normals = model_buffers_vulkan_.normals;

  model_buffers_.hits = viewContainer->hits_view;
  model_buffers_.points = viewContainer->points_view;
  model_buffers_.normals = viewContainer->normals_view;
}


RCCVulkanO1Dn::RCCVulkanO1Dn(
  rm::VulkanMapPtr map)
: rm::O1DnSimulatorVulkan(map)
{
  
}

void RCCVulkanO1Dn::setTsb(const rm::Transform& Tsb)
{
  CorrespondencesCUDA::setTsb(Tsb);
  rm::O1DnSimulatorVulkan::setTsb(Tsb);
}

void RCCVulkanO1Dn::setModel(const rm::O1DnModel& sensor_model)
{
  rm::O1DnSimulatorVulkan::setModel(sensor_model);
  model_cache_ = sensor_model;
}

void RCCVulkanO1Dn::find(const rm::Transform& Tbm_est)
{
  size_t n_new_measurements = model_cache_.size();
  if(n_new_measurements == 0)
  {
    return;
  }

  size_t n_old_measurements = model_buffers_.points.size();
  if(n_new_measurements != n_old_measurements)
  {
    rm::resize_memory_bundle<rm::VRAM_CUDA>(model_buffers_, 
      model_cache_.getHeight(), model_cache_.getWidth(), 1);

    rm::resize_memory_bundle<rm::DEVICE_LOCAL_VULKAN>(model_buffers_vulkan_, 
      model_cache_.getHeight(), model_cache_.getWidth(), 1);
    
    viewContainer.reset();
    viewContainer = std::make_unique<ViewContainer>(model_buffers_vulkan_.hits, model_buffers_vulkan_.points, model_buffers_vulkan_.normals);
  }

  simulate(Tbm_est, model_buffers_vulkan_);

  // model_buffers_.hits = model_buffers_vulkan_.hits;
  // model_buffers_.points = model_buffers_vulkan_.points;
  // model_buffers_.normals = model_buffers_vulkan_.normals;

  model_buffers_.hits = viewContainer->hits_view;
  model_buffers_.points = viewContainer->points_view;
  model_buffers_.normals = viewContainer->normals_view;
}


RCCVulkanOnDn::RCCVulkanOnDn(
  rm::VulkanMapPtr map)
: rm::OnDnSimulatorVulkan(map)
{
  
}

void RCCVulkanOnDn::setTsb(const rm::Transform& Tsb)
{
  CorrespondencesCUDA::setTsb(Tsb);
  rm::OnDnSimulatorVulkan::setTsb(Tsb);
}

void RCCVulkanOnDn::setModel(const rm::OnDnModel& sensor_model)
{
  rm::OnDnSimulatorVulkan::setModel(sensor_model);
  model_cache_ = sensor_model;
}

void RCCVulkanOnDn::find(const rm::Transform& Tbm_est)
{
  size_t n_new_measurements = model_cache_.size();
  if(n_new_measurements == 0)
  {
    return;
  }
  
  size_t n_old_measurements = model_buffers_.points.size();
  if(n_new_measurements != n_old_measurements)
  {
    rm::resize_memory_bundle<rm::VRAM_CUDA>(model_buffers_, 
      model_cache_.getHeight(), model_cache_.getWidth(), 1);

    rm::resize_memory_bundle<rm::DEVICE_LOCAL_VULKAN>(model_buffers_vulkan_, 
      model_cache_.getHeight(), model_cache_.getWidth(), 1);
    
    viewContainer.reset();
    viewContainer = std::make_unique<ViewContainer>(model_buffers_vulkan_.hits, model_buffers_vulkan_.points, model_buffers_vulkan_.normals);
  }

  simulate(Tbm_est, model_buffers_vulkan_);

  // model_buffers_.hits = model_buffers_vulkan_.hits;
  // model_buffers_.points = model_buffers_vulkan_.points;
  // model_buffers_.normals = model_buffers_vulkan_.normals;

  model_buffers_.hits = viewContainer->hits_view;
  model_buffers_.points = viewContainer->points_view;
  model_buffers_.normals = viewContainer->normals_view;
}


} // namespace rmcl