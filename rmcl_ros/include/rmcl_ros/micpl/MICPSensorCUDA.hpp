#ifndef RMCL_MICPL_SENSOR_CUDA_HPP
#define RMCL_MICPL_SENSOR_CUDA_HPP

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

  MICPSensorCUDA(rclcpp::Node::SharedPtr nh);

  void drawCorrespondences();
};

} // namespace rmcl

#endif // RMCL_MICPL_SENSOR_CUDA_HPP