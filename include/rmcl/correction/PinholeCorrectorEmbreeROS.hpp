#ifndef RMCL_PINHOLE_CORRECTOR_EMBREE_ROS_HPP
#define RMCL_PINHOLE_CORRECTOR_EMBREE_ROS_HPP

#include <ros/ros.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>

// Rmagine deps
#include <rmagine/map/EmbreeMap.hpp>
#include "PinholeCorrectorEmbree.hpp"

// RCML msgs
#include <rmcl_msgs/ScanStamped.h>

#include <memory>

namespace rmcl {

class PinholeCorrectorEmbreeROS : public PinholeCorrectorEmbree {
public:

    using Base = PinholeCorrectorEmbree;
    using Base::Base;
    using Base::setParams;

    using Base::setModel;
    // TODO
    // void setModel(const rmcl_msgs::ScanInfo& info);

    using Base::setInputData;
    void setInputData(const std::vector<float>& ranges);

    // TODO
    // void setModelAndInputData(const rmcl_msgs::Scan& scan);

    using Base::setTsb;
    void setTsb(const geometry_msgs::Transform& Tsb);
    void setTsb(const std::string& sensor_frame, 
                const std::string& base_frame);

    void setTFBuffer(std::shared_ptr<tf2_ros::Buffer> tf_buffer);

    using Base::correct;

    CorrectionResults<rmagine::RAM> correct(
        const rmagine::Memory<geometry_msgs::Pose, rmagine::RAM>& Tbms
    );

    CorrectionResults<rmagine::RAM> correct(
        const std::vector<geometry_msgs::Pose>& Tbms
    );

    CorrectionResults<rmagine::RAM> correct(
        const geometry_msgs::Pose& Tbm
    );

    CorrectionResults<rmagine::RAM> correct(
        const rmagine::Memory<geometry_msgs::Transform, rmagine::RAM>& Tbms
    );

    CorrectionResults<rmagine::RAM> correct(
        const std::vector<geometry_msgs::Transform>& Tbms
    );

    CorrectionResults<rmagine::RAM> correct(
        const geometry_msgs::Transform& Tbm
    );

private:

    // for tf
    std::shared_ptr<tf2_ros::Buffer> m_tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> m_tf_listener;
};

using PinholeCorrectorEmbreeROSPtr = std::shared_ptr<PinholeCorrectorEmbreeROS>;

} // namespace rmcl

#endif // RMCL_PINHOLE_CORRECTOR_EMBREE_ROS_HPP