#ifndef RMCL_CORRECTION_MICP_SENSOR_CPU_HPP
#define RMCL_CORRECTION_MICP_SENSOR_CPU_HPP

#include <rmcl_ros/correction/MICPSensor.hpp>
#include <rclcpp/rclcpp.hpp>

// #include <visualization_msgs/msg/marker.hpp>

namespace rmcl
{

class MICPSensorCPU
: public MICPSensor_<rmagine::RAM> 
{
public:
  using Base = MICPSensor_<rmagine::RAM>;

  MICPSensorCPU(rclcpp::Node::SharedPtr nh);

  void drawCorrespondences();
};

} // namespace rmcl

#endif // RMCL_CORRECTION_MICP_SENSOR_CPU_HPP