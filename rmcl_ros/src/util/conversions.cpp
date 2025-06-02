#include "rmcl_ros/util/conversions.h"
#include <rmcl_ros/util/scan_operations.h>

namespace rm = rmagine;

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

template<typename ModelT>
void convert(
  sensor_msgs::msg::PointCloud2& cloud,
  const std_msgs::msg::Header& header,
  const ModelT& model,
  const rmcl_msgs::msg::RangeData& data,
  bool dense
)
{
  // Create pointfields
  sensor_msgs::msg::PointField field_point_x;
  sensor_msgs::msg::PointField field_point_y;
  sensor_msgs::msg::PointField field_point_z;
  sensor_msgs::msg::PointField field_range;
  sensor_msgs::msg::PointField field_mask;
  sensor_msgs::msg::PointField field_normal_x;
  sensor_msgs::msg::PointField field_normal_y;
  sensor_msgs::msg::PointField field_normal_z;
  sensor_msgs::msg::PointField field_color;
  sensor_msgs::msg::PointField field_stamp;
  sensor_msgs::msg::PointField field_intensity;
  sensor_msgs::msg::PointField field_label;

  const bool has_mask = data.mask.size() > 0;
  const bool has_normals = data.normals.size() > 0;
  const bool has_colors = data.colors.size() > 0;
  const bool has_stamps = data.stamps.size() > 0;
  const bool has_intensities = data.intensities.size() > 0;
  const bool has_labels = data.labels.size() > 0;

  cloud.fields.resize(0);
  size_t current_offset = 0;

  // add point field
  { 
    field_point_x.name = "x";
    field_point_x.offset = current_offset;
    field_point_x.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field_point_x.count = 1;
    cloud.fields.push_back(field_point_x);
    current_offset += 4;
  }

  {
    field_point_y.name = "y";
    field_point_y.offset = current_offset;
    field_point_y.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field_point_y.count = 1;
    cloud.fields.push_back(field_point_y);
    current_offset += 4;
  }

  {
    field_point_z.name = "z";
    field_point_z.offset = current_offset;
    field_point_z.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field_point_z.count = 1;
    cloud.fields.push_back(field_point_z);
    current_offset += 4;
  }

  {
    field_range.name = "range";
    field_range.offset = current_offset;
    field_range.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field_range.count = 1;
    cloud.fields.push_back(field_range);
    current_offset += 4;
  }

  if(has_mask && !dense)
  {
    {
      field_mask.name = "mask";
      field_mask.offset = current_offset;
      field_mask.datatype = sensor_msgs::msg::PointField::INT8;
      field_mask.count = 1;
      cloud.fields.push_back(field_mask);
      current_offset += 1;
    }
  }

  if(has_normals)
  {
    {
      field_normal_x.name = "normal_x";
      field_normal_x.offset = current_offset;
      field_normal_x.datatype = sensor_msgs::msg::PointField::FLOAT32;
      field_normal_x.count = 1;
      cloud.fields.push_back(field_normal_x);
      current_offset += 4;
    }

    {
      field_normal_y.name = "normal_y";
      field_normal_y.offset = current_offset;
      field_normal_y.datatype = sensor_msgs::msg::PointField::FLOAT32;
      field_normal_y.count = 1;
      cloud.fields.push_back(field_normal_y);
      current_offset += 4;
    }

    {
      field_normal_z.name = "normal_z";
      field_normal_z.offset = current_offset;
      field_normal_z.datatype = sensor_msgs::msg::PointField::FLOAT32;
      field_normal_z.count = 1;
      cloud.fields.push_back(field_normal_z);
      current_offset += 4;
    }
  }

  if(has_colors)
  {
    {
      field_color.name = "rgba";
      field_color.offset = current_offset;
      field_color.datatype = sensor_msgs::msg::PointField::FLOAT32;
      field_color.count = 4;
      cloud.fields.push_back(field_color);
      current_offset += 16;
    }
  }

  if(has_stamps)
  {
    {
      field_stamp.name = "time";
      field_stamp.offset = current_offset;
      field_stamp.datatype = sensor_msgs::msg::PointField::FLOAT32;
      field_stamp.count = 1;
      cloud.fields.push_back(field_stamp);
      current_offset += 4;
    }
  }

  if(has_intensities)
  {
    {
      field_intensity.name = "intensity";
      field_intensity.offset = current_offset;
      field_intensity.datatype = sensor_msgs::msg::PointField::FLOAT32;
      field_intensity.count = 1;
      cloud.fields.push_back(field_intensity);
      current_offset += 4;
    }
  }

  if(has_labels)
  {
    {
      field_label.name = "label";
      field_label.offset = current_offset;
      field_label.datatype = sensor_msgs::msg::PointField::UINT32;
      field_label.count = 1;
      cloud.fields.push_back(field_label);
      current_offset += 4;
    }
  }

  cloud.header = header;
  cloud.is_dense = dense;
  cloud.point_step = current_offset;

  if(dense)
  {
    cloud.width = model.getWidth();
    cloud.height = model.getHeight();
  } else {
    cloud.width = 0; // this is determined at runtime
    cloud.height = 1;
  }

  cloud.row_step = cloud.point_step * model.getWidth();
  cloud.data.reserve(cloud.height * cloud.row_step);

  size_t num_valid = 0;

  for(size_t vid = 0; vid < model.getHeight(); vid++)
  {
    for(size_t hid = 0; hid < model.getWidth(); hid++)
    {
      const unsigned int pid = model.getBufferId(vid, hid);

      if(has_mask && dense)
      {
        if(!data.mask[pid])
        {
          continue;
        }
      }

      const float range = data.ranges[pid];
      rm::Vector point;

      if(model.range.inside(range))
      {
        point = model.getDirection(vid, hid) * range + model.getOrigin(vid, hid); 
      } else {
        if(dense)
        {
          continue;
        }

        if(range > model.range.max)
        {
          point.x = std::numeric_limits<float>::infinity();
          point.y = std::numeric_limits<float>::infinity();
          point.z = std::numeric_limits<float>::infinity();
        } else if(range < model.range.min) {
          point.x = 0.0;
          point.y = 0.0;
          point.z = 0.0;
        }
      }

      { // PX
        const uint8_t* tmp_data = reinterpret_cast<const uint8_t*>(&point.x);
        cloud.data.push_back(tmp_data[0]);
        cloud.data.push_back(tmp_data[1]);
        cloud.data.push_back(tmp_data[2]);
        cloud.data.push_back(tmp_data[3]);
      }

      {
        const uint8_t* tmp_data = reinterpret_cast<const uint8_t*>(&point.y);
        cloud.data.push_back(tmp_data[0]);
        cloud.data.push_back(tmp_data[1]);
        cloud.data.push_back(tmp_data[2]);
        cloud.data.push_back(tmp_data[3]);
      }

      {
        const uint8_t* tmp_data = reinterpret_cast<const uint8_t*>(&point.z);
        cloud.data.push_back(tmp_data[0]);
        cloud.data.push_back(tmp_data[1]);
        cloud.data.push_back(tmp_data[2]);
        cloud.data.push_back(tmp_data[3]);
      }

      {
        const uint8_t* tmp_data = reinterpret_cast<const uint8_t*>(&range);
        cloud.data.push_back(tmp_data[0]);
        cloud.data.push_back(tmp_data[1]);
        cloud.data.push_back(tmp_data[2]);
        cloud.data.push_back(tmp_data[3]);
      }
      
      if(has_mask)
      {
        const uint8_t mask_val = (data.mask[pid] ? 1 : 0);
        cloud.data.push_back(mask_val);
      }

      if(has_normals)
      {
        { // NX
          const uint8_t* tmp_data = reinterpret_cast<const uint8_t*>(&data.normals[pid].x);
          cloud.data.push_back(tmp_data[0]);
          cloud.data.push_back(tmp_data[1]);
          cloud.data.push_back(tmp_data[2]);
          cloud.data.push_back(tmp_data[3]);
        }

        { // NY
          const uint8_t* tmp_data = reinterpret_cast<const uint8_t*>(&data.normals[pid].y);
          cloud.data.push_back(tmp_data[0]);
          cloud.data.push_back(tmp_data[1]);
          cloud.data.push_back(tmp_data[2]);
          cloud.data.push_back(tmp_data[3]);
        }

        { // NY
          const uint8_t* tmp_data = reinterpret_cast<const uint8_t*>(&data.normals[pid].z);
          cloud.data.push_back(tmp_data[0]);
          cloud.data.push_back(tmp_data[1]);
          cloud.data.push_back(tmp_data[2]);
          cloud.data.push_back(tmp_data[3]);
        }
      }

      if(has_colors)
      {
        { // CR
          const uint8_t* tmp_data = reinterpret_cast<const uint8_t*>(&data.colors[pid].r);
          cloud.data.push_back(tmp_data[0]);
          cloud.data.push_back(tmp_data[1]);
          cloud.data.push_back(tmp_data[2]);
          cloud.data.push_back(tmp_data[3]);
        }

        { // CG
          const uint8_t* tmp_data = reinterpret_cast<const uint8_t*>(&data.colors[pid].g);
          cloud.data.push_back(tmp_data[0]);
          cloud.data.push_back(tmp_data[1]);
          cloud.data.push_back(tmp_data[2]);
          cloud.data.push_back(tmp_data[3]);
        }

        { // CB
          const uint8_t* tmp_data = reinterpret_cast<const uint8_t*>(&data.colors[pid].b);
          cloud.data.push_back(tmp_data[0]);
          cloud.data.push_back(tmp_data[1]);
          cloud.data.push_back(tmp_data[2]);
          cloud.data.push_back(tmp_data[3]);
        }

        { // CA
          const uint8_t* tmp_data = reinterpret_cast<const uint8_t*>(&data.colors[pid].a);
          cloud.data.push_back(tmp_data[0]);
          cloud.data.push_back(tmp_data[1]);
          cloud.data.push_back(tmp_data[2]);
          cloud.data.push_back(tmp_data[3]);
        }
      }

      if(has_stamps)
      {
        { // stamp
          const uint8_t* tmp_data = reinterpret_cast<const uint8_t*>(&data.stamps[pid]);
          cloud.data.push_back(tmp_data[0]);
          cloud.data.push_back(tmp_data[1]);
          cloud.data.push_back(tmp_data[2]);
          cloud.data.push_back(tmp_data[3]);
        }
      }

      if(has_intensities)
      {
        { // intensity
          const uint8_t* tmp_data = reinterpret_cast<const uint8_t*>(&data.intensities[pid]);
          cloud.data.push_back(tmp_data[0]);
          cloud.data.push_back(tmp_data[1]);
          cloud.data.push_back(tmp_data[2]);
          cloud.data.push_back(tmp_data[3]);
        }
      }

      if(has_labels)
      {
        { // intensity
          const uint8_t* tmp_data = reinterpret_cast<const uint8_t*>(&data.labels[pid]);
          cloud.data.push_back(tmp_data[0]);
          cloud.data.push_back(tmp_data[1]);
          cloud.data.push_back(tmp_data[2]);
          cloud.data.push_back(tmp_data[3]);
        }
      }

      num_valid++;
    }
  }

  cloud.width = num_valid / cloud.height;
}

void convert(
  sensor_msgs::msg::PointCloud2& cloud,
  const std_msgs::msg::Header& header,
  const rmcl_msgs::msg::ScanInfo& info,
  const rmcl_msgs::msg::RangeData& data,
  bool dense)
{
  rm::SphericalModel model;
  convert(info, model);
  convert(cloud, header, model, data, dense);
}

void convert(
  sensor_msgs::msg::PointCloud2& cloud,
  const std_msgs::msg::Header& header,
  const rmcl_msgs::msg::DepthInfo& info,
  const rmcl_msgs::msg::RangeData& data,
  bool dense)
{
  rm::PinholeModel model;
  convert(info, model);
  convert(cloud, header, model, data, dense);
}

void convert(
    sensor_msgs::msg::PointCloud2& cloud,
    const std_msgs::msg::Header& header,
    const rmcl_msgs::msg::O1DnInfo& info,
    const rmcl_msgs::msg::RangeData& data,
    bool dense)
{
  rm::O1DnModel model;
  convert(info, model);
  convert(cloud, header, model, data, dense);
}

void convert(
    sensor_msgs::msg::PointCloud2& cloud,
    const std_msgs::msg::Header& header,
    const rmcl_msgs::msg::OnDnInfo& info,
    const rmcl_msgs::msg::RangeData& data,
    bool dense)
{
  rm::OnDnModel model;
  convert(info, model);
  convert(cloud, header, model, data, dense);
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

void estimateModelAndData(
  std_msgs::msg::Header& header,
  rmcl_msgs::msg::O1DnInfo& info,
  rmcl_msgs::msg::RangeData& data,
  const sensor_msgs::msg::PointCloud2& cloud)
{
  const size_t pc_width = cloud.width;
  const size_t pc_height = cloud.height;

  info.width  = pc_width;
  info.height = pc_height;
  info.orig.x = 0.0;
  info.orig.y = 0.0;
  info.orig.z = 0.0;
  info.dirs.resize(info.width * info.height);

  const sensor_msgs::msg::PointField* field_x = NULL;
  const sensor_msgs::msg::PointField* field_y = NULL;
  const sensor_msgs::msg::PointField* field_z = NULL;
  // const bool 
  const sensor_msgs::msg::PointField* field_mask = NULL;
  const sensor_msgs::msg::PointField* field_normal_x = NULL;
  const sensor_msgs::msg::PointField* field_normal_y = NULL;
  const sensor_msgs::msg::PointField* field_normal_z = NULL;
  const sensor_msgs::msg::PointField* field_color = NULL;
  bool is_rgba = false;
  const sensor_msgs::msg::PointField* field_stamp = NULL;
  const sensor_msgs::msg::PointField* field_intensity = NULL;
  const sensor_msgs::msg::PointField* field_label = NULL;

  for(const sensor_msgs::msg::PointField& field : cloud.fields)
  {
    if(field.name == "x")
    {
      field_x = &field;
      data.ranges.resize(info.width * info.height);
    }
    else if(field.name == "y")
    {
      field_y = &field;
    }
    else if(field.name == "z")
    {
      field_z = &field;
    }
    else if(field.name == "mask")
    {
      field_mask = &field;
      data.mask.resize(info.width * info.height);
    }
    else if(field.name == "normal_x")
    {
      field_normal_x = &field;
      data.normals.resize(info.width * info.height);
    }
    else if(field.name == "normal_y")
    {
      field_normal_y = &field;
    }
    else if(field.name == "normal_z")
    {
      field_normal_z = &field;
    }
    else if(field.name == "rgb")
    {
      field_color = &field;
      data.colors.resize(info.width * info.height);
    }
    else if(field.name == "rgba")
    {
      field_color = &field;
      data.colors.resize(info.width * info.height);
      is_rgba = true;
    }
    else if(field.name == "time")
    {
      field_stamp = &field;
      data.stamps.resize(info.width * info.height);
    }
    else if(field.name == "intensity")
    {
      field_intensity = &field;
      data.intensities.resize(info.width * info.height);
    }
    else if(field.name == "label")
    {
      field_label = &field;
      data.labels.resize(info.width * info.height);
    }
  }

  if(field_x == NULL || field_y == NULL || field_z == NULL)
  {
    throw std::runtime_error("NO POINT FIELD IN CLOUD!");
  }

  for(size_t tgt_i = 0; tgt_i < info.height; tgt_i++)
  {
    const size_t src_i = tgt_i;
    const uint8_t* row = &cloud.data[src_i * cloud.row_step];

    for(size_t tgt_j = 0; tgt_j < info.width; tgt_j++)
    {
      const size_t src_j = tgt_j;
      const uint8_t* data_ptr = &row[src_j * cloud.point_step];
      const size_t buffer_id = tgt_i * info.width + tgt_j;

      float x, y, z;
      if (field_x->datatype == sensor_msgs::msg::PointField::FLOAT32)
      {
        // Float
        x = *reinterpret_cast<const float*>(data_ptr + field_x->offset);
        y = *reinterpret_cast<const float*>(data_ptr + field_y->offset);
        z = *reinterpret_cast<const float*>(data_ptr + field_z->offset);
      }
      else if (field_x->datatype == sensor_msgs::msg::PointField::FLOAT64)
      {
        // Double
        x = *reinterpret_cast<const double*>(data_ptr + field_x->offset);
        y = *reinterpret_cast<const double*>(data_ptr + field_y->offset);
        z = *reinterpret_cast<const double*>(data_ptr + field_z->offset);
      }
      else
      {
        throw std::runtime_error("Field X has unknown DataType. Check Topic of PC");
      }

      if(std::isfinite(x) && std::isfinite(y) && std::isfinite(z))
      {
        rm::Vector ps_s = rm::Vector{x, y, z};
        const float range = ps_s.l2norm();
        ps_s /= range;
        info.dirs[buffer_id].x = ps_s.x;
        info.dirs[buffer_id].y = ps_s.y;
        info.dirs[buffer_id].z = ps_s.z;
        data.ranges[buffer_id] = range;
      }
      else
      {
        info.dirs[buffer_id].x = 0.0;
        info.dirs[buffer_id].y = 0.0;
        info.dirs[buffer_id].z = 0.0;
        data.ranges[buffer_id] = 0.0;
      }

      if(field_mask)
      {
        data.ranges[buffer_id] = *(data_ptr + field_mask->offset);
      }

      if(field_normal_x)
      {
        geometry_msgs::msg::Point32 normal;
        if(field_normal_x->datatype == sensor_msgs::msg::PointField::FLOAT32)
        {
          // Float
          normal.x = *reinterpret_cast<const float*>(data_ptr + field_normal_x->offset);
          normal.y = *reinterpret_cast<const float*>(data_ptr + field_normal_y->offset);
          normal.z = *reinterpret_cast<const float*>(data_ptr + field_normal_z->offset);
        }
        else if(field_normal_x->datatype == sensor_msgs::msg::PointField::FLOAT64)
        {
          // Double
          normal.x = *reinterpret_cast<const double*>(data_ptr + field_normal_x->offset);
          normal.y = *reinterpret_cast<const double*>(data_ptr + field_normal_y->offset);
          normal.z = *reinterpret_cast<const double*>(data_ptr + field_normal_z->offset);
        }
        data.normals[buffer_id] = normal;
      }

      if(field_color)
      {
        std_msgs::msg::ColorRGBA color;

        color.r = *reinterpret_cast<const float*>(data_ptr + field_color->offset);
        color.g = *reinterpret_cast<const float*>(data_ptr + field_color->offset + 4);
        color.b = *reinterpret_cast<const float*>(data_ptr + field_color->offset + 8);
        
        if(is_rgba)
        {
          color.a = *reinterpret_cast<const float*>(data_ptr + field_color->offset + 12);
        } else {
          color.a = 1.0;
        }

        data.colors[buffer_id] = color;
      }

      if(field_stamp)
      {
        data.stamps[buffer_id] = *reinterpret_cast<const float*>(data_ptr + field_stamp->offset);
      }

      if(field_intensity)
      {
        data.intensities[buffer_id] = *reinterpret_cast<const float*>(data_ptr + field_intensity->offset);
      }

      if(field_label)
      {
        data.labels[buffer_id] = *reinterpret_cast<const uint32_t*>(data_ptr + field_label->offset);
      }
    }
  }
}

} // namespace rmcl 