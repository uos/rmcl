#ifndef RMCL_LIDAR_CORRECTOR_OPTIX_ROS_HPP
#define RMCL_LIDAR_CORRECTOR_OPTIX_ROS_HPP

#include <ros/ros.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>

// Rmagine deps
#include <rmagine/map/EmbreeMap.hpp>

// Internal deps
#include "LiDARCorrectorOptix.hpp"

// RCML msgs
#include <rmcl_msgs/ScanStamped.h>

#include <memory>

namespace rmcl
{

class LiDARCorrectorOptixROS : public LiDARCorrectorOptix
{
public:
    using Base = LiDARCorrectorOptix;

    using Base::Base;
    using Base::setParams;

    using Base::setModel;
    void setModel(const rmcl_msgs::ScanInfo& info);

    using Base::setInputData;
    void setInputData(const std::vector<float>& ranges);

    void setModelAndInputData(const rmcl_msgs::Scan& scan);

    using Base::setTsb;
    void setTsb(const geometry_msgs::Transform& Tsb);

    void setTsb(const std::string& sensor_frame, 
        const std::string& base_frame);

    void setTFBuffer(std::shared_ptr<tf2_ros::Buffer> tf_buffer);

    using Base::correct;

    CorrectionResults<rmagine::VRAM_CUDA> correct(
        const rmagine::Memory<geometry_msgs::Pose, rmagine::RAM>& Tbms
    );

    CorrectionResults<rmagine::VRAM_CUDA> correct(
        const std::vector<geometry_msgs::Pose>& Tbms
    );

    CorrectionResults<rmagine::VRAM_CUDA> correct(
        const geometry_msgs::Pose& Tbm
    );

    CorrectionResults<rmagine::VRAM_CUDA> correct(
        const rmagine::Memory<geometry_msgs::Transform, rmagine::RAM>& Tbms
    );

    CorrectionResults<rmagine::VRAM_CUDA> correct(
        const std::vector<geometry_msgs::Transform>& Tbms
    );

    CorrectionResults<rmagine::VRAM_CUDA> correct(
        const geometry_msgs::Transform& Tbm
    );

private:
    // for tf
    std::shared_ptr<tf2_ros::Buffer> m_tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> m_tf_listener;
};

using LiDARCorrectorOptixROSPtr = std::shared_ptr<LiDARCorrectorOptixROS>;

} // namespace rmcl

#endif // RMCL_LIDAR_CORRECTOR_OPTIX_ROS_HPP