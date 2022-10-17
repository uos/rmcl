/*
 * Copyright (c) 2022, University Osnabrück
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University Osnabrück nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL University Osnabrück BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file
 * 
 * @brief PinholeCorrectorEmbreeROS
 *
 * @date 03.10.2022
 * @author Alexander Mock
 * 
 * @copyright Copyright (c) 2022, University Osnabrück. All rights reserved.
 * This project is released under the 3-Clause BSD License.
 * 
 */

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
#include <rmcl_msgs/Depth.h>

#include <memory>

namespace rmcl {

class PinholeCorrectorEmbreeROS : public PinholeCorrectorEmbree {
public:

    using Base = PinholeCorrectorEmbree;
    using Base::Base;
    using Base::setParams;

    using Base::setModel;
    void setModel(const rmcl_msgs::DepthInfo& info);

    using Base::setInputData;
    void setInputData(const std::vector<float>& ranges);

    void setModelAndInputData(const rmcl_msgs::Depth& depth);

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