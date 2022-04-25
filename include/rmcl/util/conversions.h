#ifndef RMCL_CONVERSIONS_HPP
#define RMCL_CONVERSIONS_HPP

#include <rmcl_msgs/ScanStamped.h>
#include <rmcl_msgs/PolarCoord.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Transform.h>

#include <Eigen/Dense>
#include <rmagine/types/Memory.hpp>

#include <rmagine/math/types.h>

namespace rmcl {

inline void convert(
    const geometry_msgs::Transform& Tros,
    rmagine::Transform& Trm)
{
    Trm.R.x = Tros.rotation.x;
    Trm.R.y = Tros.rotation.y;
    Trm.R.z = Tros.rotation.z;
    Trm.R.w = Tros.rotation.w;
    Trm.t.x = Tros.translation.x;
    Trm.t.y = Tros.translation.y;
    Trm.t.z = Tros.translation.z;
}

inline void convert(
    const rmagine::Transform& Trm,
    geometry_msgs::Transform& Tros)
{
    Tros.rotation.x = Trm.R.x;
    Tros.rotation.y = Trm.R.y;
    Tros.rotation.z = Trm.R.z;
    Tros.rotation.w = Trm.R.w;
    Tros.translation.x = Trm.t.x;
    Tros.translation.y = Trm.t.y;
    Tros.translation.z = Trm.t.z;
}

geometry_msgs::Point32 polar2cartesian(
    const rmcl_msgs::PolarCoord& polar);

void convert(
    const rmcl_msgs::Scan& scan, 
    std::vector<geometry_msgs::Point32>& cloud);

void convert(
    const rmcl_msgs::ScanStamped& scan, 
    sensor_msgs::PointCloud& cloud);

void convert(
    const rmagine::Memory<Eigen::Vector3f, rmagine::RAM>& from,
    std::vector<geometry_msgs::Point32>& to);

void convert(
    const sensor_msgs::LaserScan& scan_in, 
    rmcl_msgs::ScanStamped& scan_out);

} // namespace rmcl 

#endif //RMCL_CONVERSIONS_HPP