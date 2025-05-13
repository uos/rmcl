#include "rmcl_ros/nodes/conversion/scan_to_scan.hpp"



namespace rmcl
{

ScanToScanNode::ScanToScanNode(
  const rclcpp::NodeOptions& options)
:rclcpp::Node("scan_to_scan_node", options)
{
  declareParameters();
  fetchParameters();

  pub_scan_ = this->create_publisher<rmcl_msgs::msg::ScanStamped>(
    "rmcl_scan", 10);

  sub_scan_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "scan", 10, 
    [=](const sensor_msgs::msg::LaserScan::ConstSharedPtr& msg) -> void
    { 
      scanCB(msg);
    });
  
  callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&ScanToScanNode::parametersCallback, this, std::placeholders::_1));
}

void ScanToScanNode::declareParameters()
{
  declare_parameter("skip_begin", 0);
  declare_parameter("skip_end", 0);
  declare_parameter("increment", 1);
}

void ScanToScanNode::fetchParameters()
{
  skip_begin_ = get_parameter("skip_begin").as_int();
  skip_end_ = get_parameter("skip_end").as_int();
  increment_ = get_parameter("increment").as_int();
}

rcl_interfaces::msg::SetParametersResult ScanToScanNode::parametersCallback(
  const std::vector<rclcpp::Parameter> &parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  for(const auto& param: parameters)
  {
    if(param.get_name() == "increment")
    {
      increment_ = param.as_int();
    } 
    else if(param.get_name() == "skip_begin")
    {
      skip_begin_ = param.as_int();
    }
    else if(param.get_name() == "skip_end")
    {
      skip_end_ = param.as_int();
    }
  }

  // Here update class attributes, do some actions, etc.
  return result;
}

void ScanToScanNode::initScanArray()
{
  fillEmpty(scan_.scan);
}

bool ScanToScanNode::convert(
  const sensor_msgs::msg::LaserScan::ConstSharedPtr& scan_in,
  rmcl_msgs::msg::ScanStamped& scan) const
{
  scan.header.frame_id = scan_in->header.frame_id;
  scan.header.stamp = scan_in->header.stamp;
  
  scan.scan.info.theta_min = scan_in->angle_min + skip_begin_ * scan_in->angle_increment;
  scan.scan.info.theta_inc = scan_in->angle_increment * increment_;
  scan.scan.info.theta_n = (scan_in->ranges.size() - skip_begin_ - skip_end_) / increment_;

  scan.scan.info.phi_min = 0.0;
  scan.scan.info.phi_inc = 0.0;
  scan.scan.info.phi_n = 1;
  scan.scan.info.range_max = scan_in->range_max;
  scan.scan.info.range_min = scan_in->range_min;

  scan.scan.data.ranges.resize(scan.scan.info.theta_n);

  for(size_t tgt_i=0; tgt_i<scan.scan.info.theta_n; tgt_i++)
  {
    const size_t src_i = (tgt_i * increment_) + skip_begin_;
    scan.scan.data.ranges[tgt_i] = scan_in->ranges[src_i];
  }

  return true;
}

void ScanToScanNode::scanCB(const sensor_msgs::msg::LaserScan::ConstSharedPtr& msg)
{
  scan_.header.stamp = msg->header.stamp;
  scan_.header.frame_id = msg->header.frame_id;
  if(!convert(msg, scan_))
  {
    return;
  }
  pub_scan_->publish(scan_);
}

} // namespace rmcl


#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rmcl::ScanToScanNode)
