#include "rmcl/correction/OnDnCorrectorOptixROS.hpp"
#include <rmcl/util/conversions.h>
#include <rmcl/util/scan_operations.h>
#include <rmagine/math/types.h>
#include <rmagine/types/sensor_models.h>

namespace rm = rmagine;

namespace rmcl
{

void OnDnCorrectorOptixROS::setInputData(const std::vector<float>& ranges)
{
    rm::Memory<float, rm::RAM> ranges_rm(ranges.size());
    for(size_t i=0; i<ranges.size(); i++)
    {
        ranges_rm[i] = ranges[i];
    }

    rm::Memory<float, rm::VRAM_CUDA> ranges_rm_vram = ranges_rm;
    rm::MemoryView<float, rm::VRAM_CUDA> tmp = ranges_rm_vram;
    Base::setInputData(tmp);
}

void OnDnCorrectorOptixROS::setTsb(const geometry_msgs::Transform& Tsb)
{
    rmagine::Transform Tsb_rm;
    convert(Tsb, Tsb_rm);
    Base::setTsb(Tsb_rm);
}

void OnDnCorrectorOptixROS::setTsb(
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

void OnDnCorrectorOptixROS::setTFBuffer(
    std::shared_ptr<tf2_ros::Buffer> tf_buffer)
{
    m_tf_buffer = tf_buffer;
}

CorrectionResults<rm::VRAM_CUDA> OnDnCorrectorOptixROS::correct(
    const rmagine::Memory<geometry_msgs::Pose, rm::RAM>& Tbms)
{
    rm::Memory<rm::Transform, rm::RAM_CUDA> Tbms_rm(Tbms.size());

    for(size_t i=0; i<Tbms.size(); i++)
    {
        convert(Tbms[i], Tbms_rm[i]);
    }

    // upload
    rm::Memory<rm::Transform, rm::VRAM_CUDA> Tbms_rm_vram(Tbms.size());
    copy(Tbms_rm, Tbms_rm_vram, Base::m_stream);

    return Base::correct(Tbms_rm_vram);
}

CorrectionResults<rm::VRAM_CUDA> OnDnCorrectorOptixROS::correct(
    const std::vector<geometry_msgs::Pose>& Tbms)
{
    rm::Memory<rm::Transform, rm::RAM_CUDA> Tbms_rm(Tbms.size());

    for(size_t i=0; i<Tbms.size(); i++)
    {
        convert(Tbms[i], Tbms_rm[i]);
    }

    // upload
    rm::Memory<rm::Transform, rm::VRAM_CUDA> Tbms_rm_vram(Tbms.size());
    copy(Tbms_rm, Tbms_rm_vram, Base::m_stream);

    return Base::correct(Tbms_rm_vram);
}

CorrectionResults<rm::VRAM_CUDA> OnDnCorrectorOptixROS::correct(
    const geometry_msgs::Pose& Tbm)
{
    rm::Memory<rm::Transform, rm::RAM> Tbms_rm(1);
    convert(Tbm, Tbms_rm[0]);

    // upload
    rm::Memory<rm::Transform, rm::VRAM_CUDA> Tbms_rm_vram(1);
    copy(Tbms_rm, Tbms_rm_vram, Base::m_stream);

    return Base::correct(Tbms_rm_vram);
}

CorrectionResults<rm::VRAM_CUDA> OnDnCorrectorOptixROS::correct(
    const rmagine::Memory<geometry_msgs::Transform, rm::RAM>& Tbms)
{
    rm::Memory<rm::Transform, rm::RAM_CUDA> Tbms_rm(Tbms.size());

    for(size_t i=0; i<Tbms.size(); i++)
    {
        convert(Tbms[i], Tbms_rm[i]);
    }

    // upload
    rm::Memory<rm::Transform, rm::VRAM_CUDA> Tbms_rm_vram(Tbms.size());
    copy(Tbms_rm, Tbms_rm_vram, Base::m_stream);

    return Base::correct(Tbms_rm_vram);
}

CorrectionResults<rm::VRAM_CUDA> OnDnCorrectorOptixROS::correct(
    const std::vector<geometry_msgs::Transform>& Tbms)
{
    rm::Memory<rm::Transform, rm::RAM_CUDA> Tbms_rm(Tbms.size());

    for(size_t i=0; i<Tbms.size(); i++)
    {
        convert(Tbms[i], Tbms_rm[i]);
    }

    // upload
    rm::Memory<rm::Transform, rm::VRAM_CUDA> Tbms_rm_vram(Tbms.size());
    copy(Tbms_rm, Tbms_rm_vram, Base::m_stream);

    return Base::correct(Tbms_rm_vram);
}

CorrectionResults<rm::VRAM_CUDA> OnDnCorrectorOptixROS::correct(
    const geometry_msgs::Transform& Tbm)
{
    rm::Memory<rm::Transform, rm::RAM> Tbms_rm(1);
    convert(Tbm, Tbms_rm[0]);

    // upload
    rm::Memory<rm::Transform, rm::VRAM_CUDA> Tbms_rm_vram(1);
    copy(Tbms_rm, Tbms_rm_vram, Base::m_stream);

    return Base::correct(Tbms_rm_vram);
}

} // namespace rmcl