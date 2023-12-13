#include "rmcl/correction/OnDnCorrectorEmbreeROS.hpp"
#include <rmcl/util/conversions.h>
#include <rmcl/util/scan_operations.h>
#include <rmagine/math/types.h>
#include <rmagine/types/sensor_models.h>

using namespace rmagine;

namespace rmcl
{

// void OnDnCorrectorEmbreeROS::setModel(const rmcl_msgs::msg::ScanInfo& info)
// {
//     SphericalModel model;
//     convert(info, model);
//     Base::setModel(model);
// }

void OnDnCorrectorEmbreeROS::setInputData(
    const std::vector<float>& ranges)
{
    Memory<float, RAM> data(ranges.size());
    for(size_t i=0; i<data.size(); i++)
    {
        data[i] = ranges[i];
    }
    Base::setInputData(data);
}

// void OnDnCorrectorEmbreeROS::setModelAndInputData(
//     const rmcl_msgs::msg::Scan& scan)
// {
//     setModel(scan.info);
//     setInputData(scan.ranges);
// }

void OnDnCorrectorEmbreeROS::setTsb(const geometry_msgs::msg::Transform& Tsb)
{
    rmagine::Transform Tsb_rm;
    convert(Tsb, Tsb_rm);
    Base::setTsb(Tsb_rm);
}

void OnDnCorrectorEmbreeROS::setTsb(
    const std::string& sensor_frame, 
    const std::string& base_frame)
{
    if(!m_tf_buffer)
    {
        // TODO: if m_tf_buffer is required and we cannot construct it
        // -> put it to constructor
        auto log = rclcpp::get_logger("rmcl::OnDnCorrectorEmbreeROS");
        RCLCPP_WARN(log, "NO TF BUFFER");
        return;
    }

    rmagine::Transform Tsb;
    geometry_msgs::msg::TransformStamped Tsb_ros;
    try {
        Tsb_ros = m_tf_buffer->lookupTransform(base_frame, sensor_frame, tf2::TimePointZero);
    }
    catch (tf2::TransformException &ex) {
        auto log = rclcpp::get_logger("rmcl::OnDnCorrectorEmbreeROS");

        RCLCPP_WARN(log, "%s", ex.what());
        RCLCPP_WARN_STREAM(log, "Source: " << sensor_frame << ", Target: " << base_frame);
        RCLCPP_WARN_STREAM(log, "Setting Tsb to identity");
        
        Tsb.setIdentity();
        Base::setTsb(Tsb);
        return;
    }

    // everything went fine
    convert(Tsb_ros.transform, Tsb);
    Base::setTsb(Tsb);
}

void OnDnCorrectorEmbreeROS::setTFBuffer(std::shared_ptr<tf2_ros::Buffer> tf_buffer)
{
    m_tf_buffer = tf_buffer;
}

CorrectionResults<rmagine::RAM> OnDnCorrectorEmbreeROS::correct(
    const rmagine::Memory<geometry_msgs::msg::Pose, rmagine::RAM>& Tbms)
{
    Memory<Transform, RAM> Tbms_rm(Tbms.size());

    for(size_t i=0; i<Tbms.size(); i++)
    {
        convert(Tbms[i], Tbms_rm[i]);
    }

    return Base::correct(Tbms_rm);
}

CorrectionResults<rmagine::RAM> OnDnCorrectorEmbreeROS::correct(
    const std::vector<geometry_msgs::msg::Pose>& Tbms)
{
    Memory<Transform, RAM> Tbms_rm(Tbms.size());

    for(size_t i=0; i<Tbms.size(); i++)
    {
        convert(Tbms[i], Tbms_rm[i]);
    }

    return Base::correct(Tbms_rm);
}

CorrectionResults<rmagine::RAM> OnDnCorrectorEmbreeROS::correct(
        const geometry_msgs::msg::Pose& Tbm)
{
    Memory<Transform, RAM> Tbms_rm(1);
    convert(Tbm, Tbms_rm[0]);
    return Base::correct(Tbms_rm);
}

CorrectionResults<rmagine::RAM> OnDnCorrectorEmbreeROS::correct(
    const rmagine::Memory<geometry_msgs::msg::Transform, rmagine::RAM>& Tbms)
{
    Memory<Transform, RAM> Tbms_rm(Tbms.size());

    for(size_t i=0; i<Tbms.size(); i++)
    {
        convert(Tbms[i], Tbms_rm[i]);
    }

    return Base::correct(Tbms_rm);
}

CorrectionResults<rmagine::RAM> OnDnCorrectorEmbreeROS::correct(
    const std::vector<geometry_msgs::msg::Transform>& Tbms)
{
    Memory<Transform, RAM> Tbms_rm(Tbms.size());

    for(size_t i=0; i<Tbms.size(); i++)
    {
        convert(Tbms[i], Tbms_rm[i]);
    }

    return Base::correct(Tbms_rm);
}

CorrectionResults<rmagine::RAM> OnDnCorrectorEmbreeROS::correct(
        const geometry_msgs::msg::Transform& Tbm)
{
    Memory<Transform, RAM> Tbms_rm(1);
    convert(Tbm, Tbms_rm[0]);
    return Base::correct(Tbms_rm);
}



} // namespace rmcl