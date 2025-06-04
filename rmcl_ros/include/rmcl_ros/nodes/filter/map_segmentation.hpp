#ifndef RMCL_FILTER_MAP_SEGMENTATION_HPP
#define RMCL_FILTER_MAP_SEGMENTATION_HPP

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

// RCML msgs
#include <rmcl_msgs/msg/o1_dn_stamped.hpp>

#include <memory>

namespace rmcl
{

/**
 * The struct and the description of the segmented point as it 
 * is stored in a sensor_msgs/PointCloud2
 */
struct SegmentationPoint
{
  float x;
  float y;
  float z;

  static std::vector<sensor_msgs::msg::PointField> Fields()
  {
    std::vector<sensor_msgs::msg::PointField> fields;
    fields.reserve(3);
    { // PX
      sensor_msgs::msg::PointField field_point_x;
      field_point_x.name = "x";
      field_point_x.offset = 0;
      field_point_x.datatype = sensor_msgs::msg::PointField::FLOAT32;
      field_point_x.count = 1;
      fields.push_back(field_point_x);
    }
    { // PY
      sensor_msgs::msg::PointField field_point_y;
      field_point_y.name = "y";
      field_point_y.offset = 4;
      field_point_y.datatype = sensor_msgs::msg::PointField::FLOAT32;
      field_point_y.count = 1;
      fields.push_back(field_point_y);
    }
    { // PZ
      sensor_msgs::msg::PointField field_point_z;
      field_point_z.name = "z";
      field_point_z.offset = 8;
      field_point_z.datatype = sensor_msgs::msg::PointField::FLOAT32;
      field_point_z.count = 1;
      fields.push_back(field_point_z);
    }
    return fields;
  }
};


inline void push_back(
  sensor_msgs::msg::PointCloud2& cloud, 
  const SegmentationPoint& p)
{
  constexpr size_t nbytes = sizeof(SegmentationPoint);
  const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&p);
  for(size_t i=0; i<nbytes; i++)
  {
    cloud.data.push_back(bytes[i]);
  }
}


class MapSegmentationNode
: public rclcpp::Node
{
public:
  explicit MapSegmentationNode(
    std::string node_name,
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  rcl_interfaces::msg::SetParametersResult reconfigureCallback(
    const std::vector<rclcpp::Parameter>& params);

protected:

  float min_dist_outlier_scan_;
  float min_dist_outlier_map_;
  std::string map_frame_;
  std::string map_file_;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_outlier_scan_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_outlier_map_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
};

} // namespace rmcl

#endif // RMCL_FILTER_MAP_SEGMENTATION_HPP