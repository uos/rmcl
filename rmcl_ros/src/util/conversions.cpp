#include "rmcl_ros/util/conversions.h"
#include <rmcl_ros/util/scan_operations.h>

namespace rmcl {

void convert(
    const sensor_msgs::msg::LaserScan& from, 
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
    const rmcl_msgs::msg::ScanInfo& from,
    rmagine::SphericalModel& to)
{
  to.phi.min = from.phi_min;
  to.phi.inc = from.phi_inc;
  to.phi.size = from.phi_n;
  to.theta.min = from.theta_min;
  to.theta.inc = from.theta_inc;
  to.theta.size = from.theta_n;
  to.range.min = from.range_min;
  to.range.max = from.range_max;
}

void convert(
    const sensor_msgs::msg::CameraInfo& from,
    rmagine::PinholeModel& to)
{
  to.width = from.width;
  to.height = from.height;
  to.f[0] = from.k[0];
  to.f[1] = from.k[4];
  to.c[0] = from.k[2];
  to.c[1] = from.k[5];
}

void convert(
    const rmcl_msgs::msg::DepthInfo& from,
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
    const sensor_msgs::msg::CameraInfo& from,
    rmcl_msgs::msg::DepthInfo& to)
{
  to.width = from.width;
  to.height = from.height;
  to.fx = from.k[0];
  to.fy = from.k[4];
  to.cx = from.k[2];
  to.cy = from.k[5];
}

void convert(
    const rmcl_msgs::msg::O1DnInfo& from,
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
    const rmcl_msgs::msg::OnDnInfo& from,
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
    const geometry_msgs::msg::Transform& Tros,
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
    geometry_msgs::msg::Transform& Tros)
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
    const geometry_msgs::msg::Pose& Pros,
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
    geometry_msgs::msg::Pose& Pros)
{
  Pros.orientation.x = Trm.R.x;
  Pros.orientation.y = Trm.R.y;
  Pros.orientation.z = Trm.R.z;
  Pros.orientation.w = Trm.R.w;
  Pros.position.x = Trm.t.x;
  Pros.position.y = Trm.t.y;
  Pros.position.z = Trm.t.z;
}

geometry_msgs::msg::Point32 polar2cartesian(
    const rmcl_msgs::msg::PolarCoord& polar)
{
  geometry_msgs::msg::Point32 p_cartesian;

  p_cartesian.x = cos(polar.phi) * cos(polar.theta);
  p_cartesian.y = cos(polar.phi) * sin(polar.theta);
  p_cartesian.z = sin(polar.phi);

  p_cartesian.x *= polar.range;
  p_cartesian.y *= polar.range;
  p_cartesian.z *= polar.range;

  return p_cartesian;
}

void convert(
    const rmcl_msgs::msg::Scan& scan, 
    std::vector<geometry_msgs::msg::Point32>& cloud)
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
        geometry_msgs::msg::Point32 p_ros;
        p_ros.x = p.x;
        p_ros.y = p.y;
        p_ros.z = p.z;
        cloud.push_back(p_ros);
      }
    }
  }
}

void convert(
    const rmcl_msgs::msg::ScanStamped& scan, 
    sensor_msgs::msg::PointCloud& cloud)
{
  cloud.header = scan.header;
  convert(scan.scan, cloud.points);
}

void convert(
    const sensor_msgs::msg::LaserScan& scan_in, 
    rmcl_msgs::msg::ScanStamped& scan_out)
{
  scan_out.header = scan_in.header;
  
  scan_out.scan.info.phi_n = 1;
  scan_out.scan.info.theta_n = scan_in.ranges.size();

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

void convert(
    const rmcl_msgs::msg::O1Dn& scan, 
    std::vector<geometry_msgs::msg::Point32>& cloud)
{
  rmagine::O1DnModel model;
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
        rmagine::Vector p = model.getDirection(vid, hid) * range + model.getOrigin(vid, hid);
        geometry_msgs::msg::Point32 p_ros;
        p_ros.x = p.x;
        p_ros.y = p.y;
        p_ros.z = p.z;
        cloud.push_back(p_ros);
      }
    }
  }
}

void convert(
    const rmcl_msgs::msg::O1DnStamped& scan, 
    sensor_msgs::msg::PointCloud& cloud)
{
  cloud.header = scan.header;
  convert(scan.o1dn, cloud.points);
}


bool convert(
  const ParamTree<rclcpp::Parameter>::SharedPtr sensor_model_params,
  rmcl_msgs::msg::ScanInfo& scan_model)
{
  if(  !sensor_model_params->exists("phi_min")
    || !sensor_model_params->exists("phi_inc")
    || !sensor_model_params->exists("phi_n")
    || !sensor_model_params->exists("theta_min")
    || !sensor_model_params->exists("theta_inc")
    || !sensor_model_params->exists("theta_n")
    || !sensor_model_params->exists("range_min")
    || !sensor_model_params->exists("range_max"))
  {
    return false;
  }

  scan_model.range_min = sensor_model_params->at("range_min")->data->as_double();
  scan_model.range_max = sensor_model_params->at("range_max")->data->as_double();
  scan_model.phi_min = sensor_model_params->at("phi_min")->data->as_double();
  scan_model.phi_inc = sensor_model_params->at("phi_inc")->data->as_double();
  scan_model.phi_n = sensor_model_params->at("phi_n")->data->as_int();
  scan_model.theta_min = sensor_model_params->at("theta_min")->data->as_double();
  scan_model.theta_inc = sensor_model_params->at("theta_inc")->data->as_double();
  scan_model.theta_n = sensor_model_params->at("theta_n")->data->as_int();

  return true;
}

bool convert(
  const ParamTree<rclcpp::Parameter>::SharedPtr sensor_model_params,
  rmcl_msgs::msg::DepthInfo& depth_model)
{
  if(  !sensor_model_params->exists("fx")
    || !sensor_model_params->exists("fy")
    || !sensor_model_params->exists("cx")
    || !sensor_model_params->exists("cy")
    || !sensor_model_params->exists("width")
    || !sensor_model_params->exists("height")
    || !sensor_model_params->exists("range_min")
    || !sensor_model_params->exists("range_max"))
  {
    return false;
  }

  depth_model.width = sensor_model_params->at("width")->data->as_int();
  depth_model.height = sensor_model_params->at("height")->data->as_int();
  depth_model.range_min = sensor_model_params->at("range_min")->data->as_double();
  depth_model.range_max = sensor_model_params->at("range_max")->data->as_double();

  depth_model.fx = sensor_model_params->at("fx")->data->as_double();
  depth_model.fy = sensor_model_params->at("fy")->data->as_double();
  depth_model.cx = sensor_model_params->at("cx")->data->as_double();
  depth_model.cy = sensor_model_params->at("cy")->data->as_double();

  return true;
}

bool convert(
  const ParamTree<rclcpp::Parameter>::SharedPtr sensor_model_params,
  rmcl_msgs::msg::O1DnInfo& o1dn_model)
{
  if(  !sensor_model_params->exists("width")
    || !sensor_model_params->exists("height")
    || !sensor_model_params->exists("range_min")
    || !sensor_model_params->exists("range_max")
    || !sensor_model_params->exists("orig")
    || !sensor_model_params->exists("dirs"))
  {
    return false;
  }
  o1dn_model.width  = sensor_model_params->at("width")->data->as_int();
  o1dn_model.height = sensor_model_params->at("height")->data->as_int();
  o1dn_model.range_min = sensor_model_params->at("range_min")->data->as_double();
  o1dn_model.range_max = sensor_model_params->at("range_max")->data->as_double();
  o1dn_model.dirs.resize(o1dn_model.width * o1dn_model.height);

  const std::vector<double> orig_data = sensor_model_params->at("orig")->data->as_double_array();
  o1dn_model.orig.x = orig_data[0];
  o1dn_model.orig.y = orig_data[1];
  o1dn_model.orig.z = orig_data[2];

  const std::vector<double> dirs_data = sensor_model_params->at("dirs")->data->as_double_array();
  for(size_t i=0; i<dirs_data.size() / 3; i++)
  {
    o1dn_model.dirs[i].x = dirs_data[i * 3 + 0];
    o1dn_model.dirs[i].y = dirs_data[i * 3 + 1];
    o1dn_model.dirs[i].z = dirs_data[i * 3 + 2];
  }

  return true;
}

bool convert(
  const ParamTree<rclcpp::Parameter>::SharedPtr sensor_model_params,
  rmcl_msgs::msg::OnDnInfo& ondn_model)
{
  if(  !sensor_model_params->exists("width")
    || !sensor_model_params->exists("height")
    || !sensor_model_params->exists("range_min")
    || !sensor_model_params->exists("range_max")
    || !sensor_model_params->exists("origs")
    || !sensor_model_params->exists("dirs"))
  {
    return false;
  }

  ondn_model.width  = sensor_model_params->at("width")->data->as_int();
  ondn_model.height = sensor_model_params->at("height")->data->as_int();
  ondn_model.range_min = sensor_model_params->at("range_min")->data->as_double();
  ondn_model.range_max = sensor_model_params->at("range_max")->data->as_double();
  ondn_model.origs.resize(ondn_model.width * ondn_model.height);
  ondn_model.dirs.resize(ondn_model.width * ondn_model.height);

  const std::vector<double> origs_data = sensor_model_params->at("origs")->data->as_double_array();
  for(size_t i=0; i<origs_data.size() / 3; i++)
  {
    ondn_model.origs[i].x = origs_data[i * 3 + 0];
    ondn_model.origs[i].y = origs_data[i * 3 + 1];
    ondn_model.origs[i].z = origs_data[i * 3 + 2];
  }

  const std::vector<double> dirs_data = sensor_model_params->at("dirs")->data->as_double_array();
  for(size_t i=0; i<dirs_data.size() / 3; i++)
  {
    ondn_model.dirs[i].x = dirs_data[i * 3 + 0];
    ondn_model.dirs[i].y = dirs_data[i * 3 + 1];
    ondn_model.dirs[i].z = dirs_data[i * 3 + 2];
  }

  return true;
}


bool convert(
  const ParamTree<rclcpp::Parameter>::SharedPtr data_params,
  rmcl_msgs::msg::RangeData& range_data)
{
  if(!data_params->exists("ranges"))
  {
    return false;
  }

  const std::vector<double> ranges = data_params->at("ranges")->data->as_double_array();
  range_data.ranges.resize(ranges.size());
  for(size_t i=0; i<ranges.size(); i++)
  {
    range_data.ranges[i] = static_cast<float>(ranges[i]);
  }

  // OPTIONALS

  // Mask points as valid (true), or invalid (false)
  if(data_params->exists("mask"))
  {
    const std::vector<bool> mask = data_params->at("mask")->data->as_bool_array();
    range_data.normals.resize(mask.size());
    for(size_t i=0; i<mask.size(); i++)
    {
      range_data.mask[i] = mask[i];
    }
  }

  // Normals at dataset (eg required for GICP)
  if(data_params->exists("normals"))
  {
    const std::vector<double> normals = data_params->at("normals")->data->as_double_array();
    range_data.normals.resize(normals.size() / 3);
    for(size_t i=0; i<normals.size() / 3; i++)
    {
      range_data.normals[i].x = normals[i * 3 + 0];
      range_data.normals[i].y = normals[i * 3 + 1];
      range_data.normals[i].z = normals[i * 3 + 2];
    }
  }

  return true;
}

} // namespace rmcl 