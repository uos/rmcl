#include "rmcl/util/conversions.h"
#include <rmcl/util/scan_operations.h>

namespace rmcl {

geometry_msgs::Point32 polar2cartesian(
    const rmcl_msgs::PolarCoord& polar)
{
    geometry_msgs::Point32 p_cartesian;

    p_cartesian.x = cos(polar.phi) * cos(polar.theta);
    p_cartesian.y = cos(polar.phi) * sin(polar.theta);
    p_cartesian.z = sin(polar.phi);

    p_cartesian.x *= polar.range;
    p_cartesian.y *= polar.range;
    p_cartesian.z *= polar.range;

    return p_cartesian;
}

void convert(
    const rmcl_msgs::Scan& scan, 
    std::vector<geometry_msgs::Point32>& cloud)
{
    rmagine::SphericalModel model;
    convert(scan.info, model);

    cloud.reserve(model.size());
    for(size_t vid = 0; vid < model.getHeight(); vid++)
    {
        for(size_t hid = 0; hid < model.getWidth(); hid++)
        {
            const unsigned int pid = model.getBufferId(vid, hid);
            const float range = scan.ranges[pid];

            if(model.range.inside(range))
            {
                rmagine::Vector p = model.getDirection(vid, hid) * range;
                geometry_msgs::Point32 p_ros;
                p_ros.x = p.x;
                p_ros.y = p.y;
                p_ros.z = p.z;
                cloud.push_back(p_ros);
            }
        }
    }
}

void convert(
    const rmcl_msgs::ScanStamped& scan, 
    sensor_msgs::PointCloud& cloud)
{
    cloud.header = scan.header;
    convert(scan.scan, cloud.points);
}

void convert(
    const rmagine::Memory<Eigen::Vector3f, rmagine::RAM>& from,
    std::vector<geometry_msgs::Point32>& to)
{
    to.resize(from.size());
    for(size_t i=0; i<from.size(); i++)
    {
        to[i].x = from[i].x();
        to[i].y = from[i].y();
        to[i].z = from[i].z();
    }
}

void convert(
    const sensor_msgs::LaserScan& scan_in, 
    rmcl_msgs::ScanStamped& scan_out)
{
    scan_out.header = scan_in.header;
    
    scan_out.scan.info.phi_N = 1;
    scan_out.scan.info.theta_N = scan_in.ranges.size();

    scan_out.scan.info.range_min = scan_in.range_min;
    scan_out.scan.info.range_max = scan_in.range_max;

    scan_out.scan.info.theta_min = scan_in.angle_min;
    scan_out.scan.info.theta_inc = scan_in.angle_increment;
    
    scan_out.scan.info.phi_min = 0.0;
    scan_out.scan.info.phi_inc = 0.1;

    scan_out.scan.ranges.resize(scan_in.ranges.size());
    for(size_t i=0; i<scan_in.ranges.size(); i++)
    {
        scan_out.scan.ranges[i] = scan_in.ranges[i];
    }
}

} // namespace rmcl 