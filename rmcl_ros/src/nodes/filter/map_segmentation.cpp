#include "rmcl_ros/nodes/filter/map_segmentation.hpp"

namespace rmcl
{

MapSegmentationNode::MapSegmentationNode(
  std::string node_name,
  const rclcpp::NodeOptions& options)
: rclcpp::Node(node_name, options)
{
  declare_parameter("map_file", "");
  declare_parameter("map_frame", "map");

  map_file_ = get_parameter("map_file").as_string();
  if(map_file_ == "")
  {
    RCLCPP_ERROR_STREAM(this->get_logger(), "'map_file' is required!");
    throw std::runtime_error("O1DnMapSegmentationEmbreeNode - 'map_file' is required");
  }

  map_frame_ = get_parameter("map_frame").as_string();

  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.name = "min_dist_outlier_scan";
    descriptor.floating_point_range.push_back(
      rcl_interfaces::msg::FloatingPointRange()
      .set__from_value(0.0)
      .set__to_value(1.0));
    descriptor.description = "Minimum distance for point to be considered as outlier from scan";
    min_dist_outlier_scan_ = this->declare_parameter(descriptor.name, 0.15, descriptor);
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.name = "min_dist_outlier_map";
    descriptor.floating_point_range.push_back(
      rcl_interfaces::msg::FloatingPointRange()
      .set__from_value(0.0)
      .set__to_value(1.0));
    descriptor.description = "Minimum distance for point to be considered as outlier from map";
    min_dist_outlier_map_ = this->declare_parameter(descriptor.name, 0.15, descriptor);
  }

  dyn_params_handler_ = this->add_on_set_parameters_callback(
    std::bind(&MapSegmentationNode::reconfigureCallback, this, std::placeholders::_1)
  );

  tf_buffer_ =
    std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ =
    std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  pub_outlier_scan_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "outlier_scan", 10);
  pub_outlier_map_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "outlier_map", 10);
}

rcl_interfaces::msg::SetParametersResult MapSegmentationNode::reconfigureCallback(
  const std::vector<rclcpp::Parameter>& params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto& param: params)
  {
    if ("min_dist_outlier_map" == param.get_name())
    {
      min_dist_outlier_map_ = param.as_double();
    }
    else if ("min_dist_outlier_scan" == param.get_name())
    {
      min_dist_outlier_scan_ = param.as_double();
    }
  }

  return result;
}

} // namespace rmcl
