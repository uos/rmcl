#ifndef RMCL_CONVERSIONS_HPP
#define RMCL_CONVERSIONS_HPP

#include <rmcl_msgs/ScanStamped.h>
#include <rmcl_msgs/ScanInfo.h>
#include <rmcl_msgs/PolarCoord.h>
#include <rmcl_msgs/DepthStamped.h>
#include <rmcl_msgs/O1DnStamped.h>
#include <rmcl_msgs/OnDnStamped.h>


#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Pose.h>

#include <rmagine/types/Memory.hpp>

#include <rmagine/math/types.h>

#include <rmagine/types/sensor_models.h>

namespace rmcl {

void convert(
    const sensor_msgs::LaserScan& from, 
    rmagine::SphericalModel& to);

void convert(
    const rmcl_msgs::ScanInfo& from,
    rmagine::SphericalModel& to);

void convert(
    const sensor_msgs::CameraInfo& from,
    rmagine::PinholeModel& to);

void convert(
    const rmcl_msgs::DepthInfo& from,
    rmagine::PinholeModel& to);

void convert(
    const sensor_msgs::CameraInfo& from,
    rmcl_msgs::DepthInfo& to);

void convert(
    const rmcl_msgs::O1DnInfo& from,
    rmagine::O1DnModel& to);

void convert(
    const rmcl_msgs::OnDnInfo& from,
    rmagine::OnDnModel& to);


void convert(
    const geometry_msgs::Transform& Tros,
    rmagine::Transform& Trm);

void convert(
    const rmagine::Transform& Trm,
    geometry_msgs::Transform& Tros);

void convert(
    const geometry_msgs::Pose& Pros,
    rmagine::Transform& Trm);

void convert(
    const rmagine::Transform& Trm,
    geometry_msgs::Pose& Pros);

geometry_msgs::Point32 polar2cartesian(
    const rmcl_msgs::PolarCoord& polar);

void convert(
    const rmcl_msgs::Scan& scan, 
    std::vector<geometry_msgs::Point32>& cloud);

void convert(
    const rmcl_msgs::ScanStamped& scan, 
    sensor_msgs::PointCloud& cloud);

void convert(
    const sensor_msgs::LaserScan& scan_in, 
    rmcl_msgs::ScanStamped& scan_out);

} // namespace rmcl 

#endif //RMCL_CONVERSIONS_HPP