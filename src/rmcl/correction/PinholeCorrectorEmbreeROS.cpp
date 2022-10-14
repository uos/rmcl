#include "rmcl/correction/PinholeCorrectorEmbreeROS.hpp"
#include <rmcl/util/conversions.h>
// #include <rmcl/util/scan_operations.h>
#include <rmagine/math/types.h>
#include <rmagine/types/sensor_models.h>

using namespace rmagine;

namespace rmcl
{

void PinholeCorrectorEmbreeROS::setModel(const rmcl_msgs::DepthInfo& info)
{
    PinholeModel model;
    convert(info, model);
    Base::setModel(model);
}

void PinholeCorrectorEmbreeROS::setInputData(
    const std::vector<float>& ranges)
{
    Memory<float, RAM> data(ranges.size());
    for(size_t i=0; i<data.size(); i++)
    {
        data[i] = ranges[i];
    }
    Base::setInputData(data);
}

void PinholeCorrectorEmbreeROS::setModelAndInputData(
    const rmcl_msgs::Depth& depth)
{
    setModel(depth.info);
    setInputData(depth.data.ranges);
}

void PinholeCorrectorEmbreeROS::setTsb(const geometry_msgs::Transform& Tsb)
{
    rmagine::Transform Tsb_rm;
    convert(Tsb, Tsb_rm);
    Base::setTsb(Tsb_rm);
}

void PinholeCorrectorEmbreeROS::setTsb(
    const std::string& sensor_frame, 
    const std::string& base_frame)
{
    if(!m_tf_buffer)
    {
        m_tf_buffer.reset(new tf2_ros::Buffer);
        m_tf_listener.reset(new tf2_ros::TransformListener(*m_tf_buffer));
    }

    rmagine::Transform Tsb;
    geometry_msgs::TransformStamped Tsb_ros;
    try {
        Tsb_ros = m_tf_buffer->lookupTransform(base_frame, sensor_frame, ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
        ROS_WARN_STREAM("Source: " << sensor_frame << ", Target: " << base_frame);
        ROS_WARN_STREAM("Setting Tsb to identity");
        
        Tsb.setIdentity();
        Base::setTsb(Tsb);
        return;
    }

    // everything went fine
    convert(Tsb_ros.transform, Tsb);
    Base::setTsb(Tsb);
}

void PinholeCorrectorEmbreeROS::setTFBuffer(std::shared_ptr<tf2_ros::Buffer> tf_buffer)
{
    m_tf_buffer = tf_buffer;
}

CorrectionResults<rmagine::RAM> PinholeCorrectorEmbreeROS::correct(
    const rmagine::Memory<geometry_msgs::Pose, rmagine::RAM>& Tbms)
{
    Memory<Transform, RAM> Tbms_rm(Tbms.size());

    for(size_t i=0; i<Tbms.size(); i++)
    {
        convert(Tbms[i], Tbms_rm[i]);
    }

    return Base::correct(Tbms_rm);
}

CorrectionResults<rmagine::RAM> PinholeCorrectorEmbreeROS::correct(
    const std::vector<geometry_msgs::Pose>& Tbms)
{
    Memory<Transform, RAM> Tbms_rm(Tbms.size());

    for(size_t i=0; i<Tbms.size(); i++)
    {
        convert(Tbms[i], Tbms_rm[i]);
    }

    return Base::correct(Tbms_rm);
}

CorrectionResults<rmagine::RAM> PinholeCorrectorEmbreeROS::correct(
        const geometry_msgs::Pose& Tbm)
{
    Memory<Transform, RAM> Tbms_rm(1);
    convert(Tbm, Tbms_rm[0]);
    return Base::correct(Tbms_rm);
}

CorrectionResults<rmagine::RAM> PinholeCorrectorEmbreeROS::correct(
    const rmagine::Memory<geometry_msgs::Transform, rmagine::RAM>& Tbms)
{
    Memory<Transform, RAM> Tbms_rm(Tbms.size());

    for(size_t i=0; i<Tbms.size(); i++)
    {
        convert(Tbms[i], Tbms_rm[i]);
    }

    return Base::correct(Tbms_rm);
}

CorrectionResults<rmagine::RAM> PinholeCorrectorEmbreeROS::correct(
    const std::vector<geometry_msgs::Transform>& Tbms)
{
    Memory<Transform, RAM> Tbms_rm(Tbms.size());

    for(size_t i=0; i<Tbms.size(); i++)
    {
        convert(Tbms[i], Tbms_rm[i]);
    }

    return Base::correct(Tbms_rm);
}

CorrectionResults<rmagine::RAM> PinholeCorrectorEmbreeROS::correct(
        const geometry_msgs::Transform& Tbm)
{
    Memory<Transform, RAM> Tbms_rm(1);
    convert(Tbm, Tbms_rm[0]);
    return Base::correct(Tbms_rm);
}



} // namespace rmcl