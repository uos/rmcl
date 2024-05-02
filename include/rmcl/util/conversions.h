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
 * @brief ROS Conversions
 *
 * @date 03.10.2022
 * @author Alexander Mock
 * 
 * @copyright Copyright (c) 2022, University Osnabrück. All rights reserved.
 * This project is released under the 3-Clause BSD License.
 * 
 */

#ifndef RMCL_CONVERSIONS_HPP
#define RMCL_CONVERSIONS_HPP

#include <rmcl_msgs/msg/scan_stamped.hpp>
#include <rmcl_msgs/msg/scan_info.hpp>
#include <rmcl_msgs/msg/polar_coord.hpp>
#include <rmcl_msgs/msg/depth_stamped.hpp>
#include <rmcl_msgs/msg/o1_dn_stamped.hpp>
#include <rmcl_msgs/msg/on_dn_stamped.hpp>


#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <rmagine/types/Memory.hpp>

#include <rmagine/math/types.h>

#include <rmagine/types/sensor_models.h>

namespace rmcl {

void convert(
    const sensor_msgs::msg::LaserScan& from, 
    rmagine::SphericalModel& to);

void convert(
    const rmcl_msgs::msg::ScanInfo& from,
    rmagine::SphericalModel& to);

void convert(
    const sensor_msgs::msg::CameraInfo& from,
    rmagine::PinholeModel& to);

void convert(
    const rmcl_msgs::msg::DepthInfo& from,
    rmagine::PinholeModel& to);

void convert(
    const sensor_msgs::msg::CameraInfo& from,
    rmcl_msgs::msg::DepthInfo& to);

void convert(
    const rmcl_msgs::msg::O1DnInfo& from,
    rmagine::O1DnModel& to);

void convert(
    const rmcl_msgs::msg::OnDnInfo& from,
    rmagine::OnDnModel& to);


void convert(
    const geometry_msgs::msg::Transform& Tros,
    rmagine::Transform& Trm);

void convert(
    const rmagine::Transform& Trm,
    geometry_msgs::msg::Transform& Tros);

void convert(
    const geometry_msgs::msg::Pose& Pros,
    rmagine::Transform& Trm);

void convert(
    const rmagine::Transform& Trm,
    geometry_msgs::msg::Pose& Pros);

geometry_msgs::msg::Point32 polar2cartesian(
    const rmcl_msgs::msg::PolarCoord& polar);

void convert(
    const rmcl_msgs::msg::Scan& scan, 
    std::vector<geometry_msgs::msg::Point32>& cloud);

void convert(
    const rmcl_msgs::msg::ScanStamped& scan, 
    sensor_msgs::msg::PointCloud& cloud);

void convert(
    const sensor_msgs::msg::LaserScan& scan_in, 
    rmcl_msgs::msg::ScanStamped& scan_out);

} // namespace rmcl 

#endif //RMCL_CONVERSIONS_HPP