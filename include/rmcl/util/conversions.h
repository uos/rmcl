#ifndef RMCL_CONVERSIONS_HPP
#define RMCL_CONVERSIONS_HPP

#include <rmcl_msgs/ScanStamped.h>
#include <rmcl_msgs/PolarCoord.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>

#include <Eigen/Dense>
#include <rmagine/types/Memory.hpp>

namespace rmcl {

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