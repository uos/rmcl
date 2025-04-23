#ifndef RMCL_CORRECTION_MICP_SENSOR_SPHERICAL_EMBREE_HPP
#define RMCL_CORRECTION_MICP_SENSOR_SPHERICAL_EMBREE_HPP

#include <rmcl_ros/correction/MICPSensorCPU.hpp>


namespace rmcl 
{

/**
 * This sensor continously corrects spherical sensors using Embree
 * outputs are corrections in RAM
 */
class MICPSensorSphericalEmbree
: public MICPSensorCPU
{
  // static spherical sensor model
  
public:
  void test();

protected:
  // static model
  rmagine::SphericalModel              model_;

private:
  

};

} // namespace rmcl

#endif // RMCL_CORRECTION_MICP_SENSOR_SPHERICAL_EMBREE_HPP