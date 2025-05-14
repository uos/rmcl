#ifndef RMCL_CORRECTION_MICP_SENSOR_CPU_HPP
#define RMCL_CORRECTION_MICP_SENSOR_CPU_HPP

#include <rmcl_ros/micpl/MICPSensor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rmagine/types/MemoryCuda.hpp>

namespace rmcl
{

class MICPSensorCUDA
: public MICPSensor_<rmagine::VRAM_CUDA> 
{
public:
  using Base = MICPSensor_<rmagine::VRAM_CUDA>;

  MICPSensorCPU(rclcpp::Node::SharedPtr nh);

  void drawCorrespondences();
};

} // namespace rmcl

#endif // RMCL_CORRECTION_MICP_SENSOR_CPU_HPP