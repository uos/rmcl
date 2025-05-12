#include "rmcl_ros/nodes/conversion/pc2_to_o1dn.hpp"

#include <rmagine/math/types.h>
#include <rmagine/util/prints.h>

#include <rmcl_ros/util/conversions.h>
#include <rmcl_ros/util/scan_operations.h>


namespace rm = rmagine;

namespace rmcl
{

Pc2ToO1DnNode::Pc2ToO1DnNode(
  const rclcpp::NodeOptions& options)
:rclcpp::Node("pcl2_to_o1dn_node", options)
{
  declareParameters();
  fetchParameters();

  pub_scan_ = this->create_publisher<rmcl_msgs::msg::O1DnStamped>(
    "rmcl_scan", 10);

  if(debug_cloud_)
  {
    pub_debug_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud>(
      "debug_cloud", 10);
  }

  tf_buffer_ =
    std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ =
    std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  sub_pcl_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "cloud", 10, 
    [=](const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) -> void
    { 
      cloudCB(msg); 
    });
  
  callback_handle_ = this->add_on_set_parameters_callback(
          std::bind(&Pc2ToO1DnNode::parametersCallback, this, std::placeholders::_1));
}

void Pc2ToO1DnNode::declareParameters()
{
  declare_parameter("sensor_frame", "");
  declare_parameter("model.range_min", 0.3);
  declare_parameter("model.range_max", 50.0);
  declare_parameter("debug_cloud", false);
  declare_parameter("height.increment", 1);
  declare_parameter("height.skip_begin", 0);
  declare_parameter("height.skip_end", 0);
  declare_parameter("width.increment", 1);
  declare_parameter("width.skip_begin", 0);
  declare_parameter("width.skip_end", 0);
}

void Pc2ToO1DnNode::fetchParameters()
{
  sensor_frame_ = get_parameter("sensor_frame").as_string();
  scan_.o1dn.info.range_min = get_parameter("model.range_min").as_double();
  scan_.o1dn.info.range_max = get_parameter("model.range_max").as_double();
  debug_cloud_ = get_parameter("debug_cloud").as_bool();
  height_increment = get_parameter("height.increment").as_int();
  height_skip_begin = get_parameter("height.skip_begin").as_int();
  height_skip_end = get_parameter("height.skip_end").as_int();
  width_increment = get_parameter("width.increment").as_int();
  width_skip_begin = get_parameter("width.skip_begin").as_int();
  width_skip_end = get_parameter("width.skip_end").as_int();
}

rcl_interfaces::msg::SetParametersResult Pc2ToO1DnNode::parametersCallback(
    const std::vector<rclcpp::Parameter> &parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  for(const auto& param: parameters)
  {
    if(param.get_name() == "height.increment")
    {
      height_increment = param.as_int();
    } 
    else if(param.get_name() == "height.skip_begin")
    {
      height_skip_begin = param.as_int();
    }
    else if(param.get_name() == "height.skip_end")
    {
      height_skip_end = param.as_int();
    }
    else if(param.get_name() == "width.increment")
    {
      width_increment = param.as_int();
    }
    else if(param.get_name() == "width.skip_begin")
    {
      width_skip_begin = param.as_int();
    }
    else if(param.get_name() == "width.skip_end")
    {
      width_skip_end = param.as_int();
    }
    else if(param.get_name() == "debug_cloud")
    {
      debug_cloud_ = param.as_bool();
      if(debug_cloud_ && !pub_debug_cloud_)
      {
        pub_debug_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud>(
          "debug_cloud", 10);
      }
      else if(!debug_cloud_ && pub_debug_cloud_)
      {
        pub_debug_cloud_.reset();
      }
    }
    else if(param.get_name() == "model.range_min")
    {
      scan_.o1dn.info.range_min = param.as_double();
    }
    else if(param.get_name() == "model.range_max")
    {
      scan_.o1dn.info.range_max = param.as_double();
    }
  }

  // Here update class attributes, do some actions, etc.
  return result;
}


bool Pc2ToO1DnNode::convert(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr& pcd,
  rmcl_msgs::msg::O1DnStamped& scan) const
{
  rm::Transform T = rm::Transform::Identity();

  if (pcd->header.frame_id != sensor_frame_)
  {
    geometry_msgs::msg::TransformStamped Tros;

    try
    {
      Tros = tf_buffer_->lookupTransform(sensor_frame_, pcd->header.frame_id,
        pcd->header.stamp);
      T.t.x = Tros.transform.translation.x;
      T.t.y = Tros.transform.translation.y;
      T.t.z = Tros.transform.translation.z;
      T.R.x = Tros.transform.rotation.x;
      T.R.y = Tros.transform.rotation.y;
      T.R.z = Tros.transform.rotation.z;
      T.R.w = Tros.transform.rotation.w;
    }
    catch (tf2::TransformException &ex)
    {
      RCLCPP_WARN(this->get_logger(), "%s", ex.what());
      return false;
    }
  }

  const size_t pcd_width = pcd->width;
  const size_t pcd_height = pcd->height;

  scan.o1dn.info.width  = (pcd_width - width_skip_begin - width_skip_end) / width_increment;
  scan.o1dn.info.height = (pcd_height - height_skip_begin - height_skip_end) / height_increment;
  scan.o1dn.info.dirs.resize(scan.o1dn.info.width * scan.o1dn.info.height);
  scan.o1dn.data.ranges.resize(scan.o1dn.info.width * scan.o1dn.info.height);
  
  scan.o1dn.info.orig.x = 0.0;
  scan.o1dn.info.orig.y = 0.0;
  scan.o1dn.info.orig.z = 0.0;

  sensor_msgs::msg::PointField field_x;
  sensor_msgs::msg::PointField field_y;
  sensor_msgs::msg::PointField field_z;

  for (size_t i = 0; i < pcd->fields.size(); i++)
  {
    if (pcd->fields[i].name == "x")
    {
      field_x = pcd->fields[i];
    }
    if (pcd->fields[i].name == "y")
    {
      field_y = pcd->fields[i];
    }
    if (pcd->fields[i].name == "z")
    {
      field_z = pcd->fields[i];
    }
  }

  for(size_t tgt_i = 0; tgt_i < scan.o1dn.info.height; tgt_i++)
  {
    const size_t src_i = tgt_i * height_increment + height_skip_begin;
    const uint8_t* row = &pcd->data[src_i * pcd->row_step];

    for(size_t tgt_j = 0; tgt_j < scan.o1dn.info.width; tgt_j++)
    {
      const size_t src_j = tgt_j * width_increment + width_skip_begin;
      const uint8_t* data_ptr = &row[src_j * pcd->point_step];
      const size_t buffer_id = tgt_i * scan.o1dn.info.width + tgt_j;

      // rmagine::Vector point;
      float x, y, z;
      if (field_x.datatype == sensor_msgs::msg::PointField::FLOAT32)
      {
        // Float
        x = *reinterpret_cast<const float*>(data_ptr + field_x.offset);
        y = *reinterpret_cast<const float*>(data_ptr + field_y.offset);
        z = *reinterpret_cast<const float*>(data_ptr + field_z.offset);
      }
      else if (field_x.datatype == sensor_msgs::msg::PointField::FLOAT64)
      {
        // Double
        x = *reinterpret_cast<const double*>(data_ptr + field_x.offset);
        y = *reinterpret_cast<const double*>(data_ptr + field_y.offset);
        z = *reinterpret_cast<const double*>(data_ptr + field_z.offset);
      }
      else
      {
        throw std::runtime_error("Field X has unknown DataType. Check Topic of pcl");
      }
      
      if(std::isfinite(x) && std::isfinite(y) && std::isfinite(z))
      {
        const rm::Vector ps_s = rm::Vector{x, y, z};
        rm::Vector ps = T * ps_s;
        const float range_est = ps.l2norm();
        ps = ps / range_est;
        scan.o1dn.data.ranges[buffer_id] = range_est;
        scan.o1dn.info.dirs[buffer_id].x = ps.x;
        scan.o1dn.info.dirs[buffer_id].y = ps.y;
        scan.o1dn.info.dirs[buffer_id].z = ps.z;
      } 
      else 
      {
        scan.o1dn.data.ranges[buffer_id] = scan.o1dn.info.range_max + 1;
        scan.o1dn.info.dirs[buffer_id].x = 0;
        scan.o1dn.info.dirs[buffer_id].y = 0;
        scan.o1dn.info.dirs[buffer_id].z = 0;
      }
    }
  }

  scan.header.stamp = pcd->header.stamp;

  return true;
}

void Pc2ToO1DnNode::cloudCB(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg)
{
  if(sensor_frame_ == "")
  {
    sensor_frame_ = msg->header.frame_id;
  }

  scan_.header.stamp = msg->header.stamp;
  scan_.header.frame_id = sensor_frame_;
  if(!convert(msg, scan_))
  {
    return;
  }

  pub_scan_->publish(scan_);

  if(debug_cloud_)
  {
    if(!pub_debug_cloud_)
    {
      // SHOULD NEVER HAPPEN
      RCLCPP_ERROR(get_logger(), "ERROR: debug cloud is true but publisher does not exist!");
    }
    sensor_msgs::msg::PointCloud cloud;
    rmcl::convert(scan_, cloud);
    cloud.header.stamp = msg->header.stamp;
    pub_debug_cloud_->publish(cloud);
  }
}

} // namespace rmcl

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rmcl::Pc2ToO1DnNode)
