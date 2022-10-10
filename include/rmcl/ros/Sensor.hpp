#ifndef RMCL_ROS_SENSOR_HPP
#define RMCL_ROS_SENSOR_HPP

#include <rmagine/types/sensor_models.h>
#include <rmagine/types/Memory.hpp>
#include <memory>

namespace rmcl
{


// using RangeModelPtr = std::shared_ptr<RangeModel>;

template<typename ModelT, typename MemT>
class RangeSensorROS
{
public:
    // return shallow copy
    // - to be thread safe:
    //  -> directly write to Memory Object 
    //  -> mutex
    rmagine::MemoryView<float, MemT> ranges();

    ModelT model();

private:
    // is this possible?
    ModelT                       m_model;
    // frequently filled by callback
    rmagine::Memory<float, MemT> m_ranges;
};



template<>
class RangeSensorROS<rmagine::SphericalModel, rmagine::RAM>
{

};



} // namespace rmcl

#include "Sensor.tcc"

#endif // RMCL_ROS_SENSOR_HPP