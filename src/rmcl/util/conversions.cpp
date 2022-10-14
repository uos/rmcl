#include "rmcl/util/conversions.h"
#include <rmcl/util/scan_operations.h>

namespace rmcl {

void convert(
    const sensor_msgs::LaserScan& from, 
    rmagine::SphericalModel& to)
{
    to.range.min = from.range_min;
    to.range.max = from.range_max;
    to.theta.size = from.ranges.size();
    to.theta.min = from.angle_min;
    to.theta.inc = from.angle_increment;
    to.phi.size = 1;
    to.phi.min = 0.0;
    to.phi.inc = 0.0;
}

void convert(
    const rmcl_msgs::ScanInfo& from,
    rmagine::SphericalModel& to)
{
    to.phi.min = from.phi_min;
    to.phi.inc = from.phi_inc;
    to.phi.size = from.phi_N;
    to.theta.min = from.theta_min;
    to.theta.inc = from.theta_inc;
    to.theta.size = from.theta_N;
    to.range.min = from.range_min;
    to.range.max = from.range_max;
}

void convert(
    const sensor_msgs::CameraInfo& from,
    rmagine::PinholeModel& to)
{
    to.width = from.width;
    to.height = from.height;
    to.f[0] = from.K[0];
    to.f[1] = from.K[4];
    to.c[0] = from.K[2];
    to.c[1] = from.K[5];
}

void convert(
    const rmcl_msgs::DepthInfo& from,
    rmagine::PinholeModel& to)
{
    to.width = from.width;
    to.height = from.height;
    to.f[0] = from.fx;
    to.f[1] = from.fy;
    to.c[0] = from.cx;
    to.c[1] = from.cy;
    to.range.min = from.range_min;
    to.range.max = from.range_max;
}

void convert(
    const sensor_msgs::CameraInfo& from,
    rmcl_msgs::DepthInfo& to)
{
    to.width = from.width;
    to.height = from.height;
    to.fx = from.K[0];
    to.fy = from.K[4];
    to.cx = from.K[2];
    to.cy = from.K[5];
}

void convert(
    const rmcl_msgs::O1DnInfo& from,
    rmagine::O1DnModel& to)
{
    to.range.min = from.range_min;
    to.range.max = from.range_max;
    to.width = from.width;
    to.height = from.height;

    to.orig.x = from.orig.x;
    to.orig.y = from.orig.y;
    to.orig.z = from.orig.z;

    to.dirs.resize(from.dirs.size());
    for(size_t i=0; i<to.dirs.size(); i++)
    {
        to.dirs[i].x = from.dirs[i].x;
        to.dirs[i].y = from.dirs[i].y;
        to.dirs[i].z = from.dirs[i].z;
    }
}

void convert(
    const rmcl_msgs::OnDnInfo& from,
    rmagine::OnDnModel& to)
{
    to.width = from.width;
    to.height = from.height;
    to.range.min = from.range_min;
    to.range.max = from.range_max;

    to.origs.resize(from.origs.size());
    for(size_t i=0; i<to.origs.size(); i++)
    {
        to.origs[i].x = from.origs[i].x;
        to.origs[i].y = from.origs[i].y;
        to.origs[i].z = from.origs[i].z;
    }

    to.dirs.resize(from.dirs.size());
    for(size_t i=0; i<to.dirs.size(); i++)
    {
        to.dirs[i].x = from.dirs[i].x;
        to.dirs[i].y = from.dirs[i].y;
        to.dirs[i].z = from.dirs[i].z;
    }
}

void convert(
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

void convert(
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

void convert(
    const geometry_msgs::Pose& Pros,
    rmagine::Transform& Trm)
{
    Trm.R.x = Pros.orientation.x;
    Trm.R.y = Pros.orientation.y;
    Trm.R.z = Pros.orientation.z;
    Trm.R.w = Pros.orientation.w;
    Trm.t.x = Pros.position.x;
    Trm.t.y = Pros.position.y;
    Trm.t.z = Pros.position.z;
}

void convert(
    const rmagine::Transform& Trm,
    geometry_msgs::Pose& Pros)
{
    Pros.orientation.x = Trm.R.x;
    Pros.orientation.y = Trm.R.y;
    Pros.orientation.z = Trm.R.z;
    Pros.orientation.w = Trm.R.w;
    Pros.position.x = Trm.t.x;
    Pros.position.y = Trm.t.y;
    Pros.position.z = Trm.t.z;
}

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
            const float range = scan.data.ranges[pid];

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

    scan_out.scan.data.ranges.resize(scan_in.ranges.size());
    for(size_t i=0; i<scan_in.ranges.size(); i++)
    {
        scan_out.scan.data.ranges[i] = scan_in.ranges[i];
    }
}

} // namespace rmcl 