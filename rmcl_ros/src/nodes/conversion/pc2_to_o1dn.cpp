#include "rmcl_ros/nodes/conversion/pc2_to_o1dn.hpp"

#include <rmagine/math/types.h>
#include <rmagine/util/prints.h>
#include <rmagine/util/StopWatch.hpp>

#include <rmcl_ros/util/conversions.h>
#include <rmcl_ros/util/scan_operations.h>


namespace rm = rmagine;

namespace rmcl
{

Pc2ToO1DnNode::Pc2ToO1DnNode(
  const rclcpp::NodeOptions& options)
:rclcpp::Node("pc2_to_o1dn_node", options)
{
  declareParameters();
  fetchParameters();

  pub_scan_ = this->create_publisher<rmcl_msgs::msg::O1DnStamped>(
    "output", 10);

  if(debug_cloud_)
  {
    pub_debug_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "~/debug_cloud", 10);
  }

  tf_buffer_ =
    std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ =
    std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  sub_pcl_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "input", 10, 
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
  filter_options_.range_min = scan_.o1dn.info.range_min;
  filter_options_.range_max = scan_.o1dn.info.range_max;

  filter_options_.height.increment = get_parameter("height.increment").as_int();
  filter_options_.height.skip_begin = get_parameter("height.skip_begin").as_int();
  filter_options_.height.skip_end = get_parameter("height.skip_end").as_int();
  filter_options_.width.increment = get_parameter("width.increment").as_int();
  filter_options_.width.skip_begin = get_parameter("width.skip_begin").as_int();
  filter_options_.width.skip_end = get_parameter("width.skip_end").as_int();
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
      filter_options_.height.increment = param.as_int();
    } 
    else if(param.get_name() == "height.skip_begin")
    {
      filter_options_.height.skip_begin = param.as_int();
    }
    else if(param.get_name() == "height.skip_end")
    {
      filter_options_.height.skip_end = param.as_int();
    }
    else if(param.get_name() == "width.increment")
    {
      filter_options_.width.increment = param.as_int();
    }
    else if(param.get_name() == "width.skip_begin")
    {
      filter_options_.width.skip_begin = param.as_int();
    }
    else if(param.get_name() == "width.skip_end")
    {
      filter_options_.width.skip_end = param.as_int();
    }
    else if(param.get_name() == "debug_cloud")
    {
      debug_cloud_ = param.as_bool();
      if(debug_cloud_ && !pub_debug_cloud_)
      {
        pub_debug_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
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
      filter_options_.range_min = scan_.o1dn.info.range_min;
    }
    else if(param.get_name() == "model.range_max")
    {
      scan_.o1dn.info.range_max = param.as_double();
      filter_options_.range_max = scan_.o1dn.info.range_max;
    }
  }

  // Here update class attributes, do some actions, etc.
  return result;
}


bool Pc2ToO1DnNode::convert(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr& pcd,
  rmcl_msgs::msg::O1DnStamped& scan) const
{
  rmcl_msgs::msg::O1DnStamped scan_unfiltered;

  scan_unfiltered.o1dn.info.range_min = scan.o1dn.info.range_min;
  scan_unfiltered.o1dn.info.range_max = scan.o1dn.info.range_max;
  
  estimateModelAndData(scan_unfiltered.header, scan_unfiltered.o1dn.info, scan_unfiltered.o1dn.data, *pcd);
  
  filter(scan.o1dn, scan_unfiltered.o1dn, filter_options_);

  return true;
}

void Pc2ToO1DnNode::cloudCB(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg)
{
  const rclcpp::Time msg_time = msg->header.stamp;
  const rclcpp::Time ros_now = this->get_clock()->now();

  const double diff_now_msg = (ros_now - msg_time).seconds();
  if(fabs(diff_now_msg) > 0.5)
  {
    RCLCPP_WARN_STREAM(this->get_logger(), "[Pc2ToO1DnNode::cloudCB] WARNING - NETWORK DELAY: (now - input msg's stamp) is far apart (" << diff_now_msg * 1000.0 << " ms).");
  }

  rm::StopWatch sw;
  sw();

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

  const double el = sw();
  // const rclcpp::Time phsical_time_2 = this->get_clock()->now();
  // const double diff_now_msg_2 = (phsical_time_2 - phsical_time_1).seconds();
  if(fabs(el) > 0.5)
  {
    RCLCPP_WARN_STREAM(this->get_logger(), "[Pc2ToO1DnNode::cloudCB] WARNING: Conversion takes too long (" << el * 1000.0 << " ms).");
  }

  // this is send directly to RMCL sensors
  pub_scan_->publish(scan_);

  if(debug_cloud_)
  {
    if(!pub_debug_cloud_)
    {
      // SHOULD NEVER HAPPEN
      RCLCPP_ERROR(get_logger(), "ERROR: debug cloud is true but publisher does not exist!");
    }
    sensor_msgs::msg::PointCloud2 cloud;
    rmcl::convert(cloud, scan_.header, scan_.o1dn.info, scan_.o1dn.data);
    pub_debug_cloud_->publish(cloud);
  }
}

} // namespace rmcl

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rmcl::Pc2ToO1DnNode)
