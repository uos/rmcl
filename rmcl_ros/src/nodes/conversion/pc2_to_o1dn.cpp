#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <rmcl_msgs/msg/o1_dn_stamped.hpp>

#include <rmcl_ros/util/conversions.h>
#include <rmcl_ros/util/scan_operations.h>

#include <rmagine/math/types.h>
#include <rmagine/util/prints.h>

#include <Eigen/Dense>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>
#include <tf2/convert.h>
#include <geometry_msgs/msg/transform_stamped.hpp>


namespace rm = rmagine;

namespace rmcl
{

class Pc2ToO1DnNode : public rclcpp::Node
{
public:
  explicit Pc2ToO1DnNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
  :rclcpp::Node("pcl2_to_o1dn_node", rclcpp::NodeOptions(options)
    .allow_undeclared_parameters(true)
    .automatically_declare_parameters_from_overrides(true))
  {
    fetchParameters();

    pub_scan_ = this->create_publisher<rmcl_msgs::msg::O1DnStamped>(
      "rmcl_scan", 10);

    if(debug_cloud)
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
  }

private:

  void fetchParameters()
  {
    if(!this->get_parameter("sensor_frame", sensor_frame))
    {
      sensor_frame = "";
    }
    
    rmcl_msgs::msg::O1DnInfo &scanner_model = scan_.o1dn.info;

    if(!this->get_parameter("model.range_min", scanner_model.range_min))
    {
      RCLCPP_ERROR_STREAM(this->get_logger(), "When specifying auto_detect_phi to false you have to provide model.phi_min");
      return;
    }
    if (!this->get_parameter("model.range_max", scanner_model.range_max))
    {
      RCLCPP_ERROR_STREAM(this->get_logger(), "When specifying auto_detect_phi to false you have to provide model.phi_max");
      return;
    }

    if(!this->get_parameter("debug_cloud", debug_cloud))
    {
      debug_cloud = false;
    }

    if(!this->get_parameter("height_increment", height_increment))
    {
      height_increment = 1;
    }

    if(!this->get_parameter("width_increment", width_increment))
    {
      width_increment = 1;
    }
  }

  bool convert(
      const sensor_msgs::msg::PointCloud2::ConstSharedPtr& pcd,
      rmcl_msgs::msg::O1DnStamped& scan) const
  {
    rm::Transform T = rm::Transform::Identity();

    if (pcd->header.frame_id != sensor_frame)
    {
      geometry_msgs::msg::TransformStamped Tros;

      try
      {
        Tros = tf_buffer_->lookupTransform(sensor_frame, pcd->header.frame_id,
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

    scan.o1dn.info.width  = pcd_width / width_increment;
    scan.o1dn.info.height = pcd_height/ height_increment;
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

    // TODO: parameter
    

    for(size_t i = 0; i < pcd_height; i += height_increment)
    {
      const uint8_t* row = &pcd->data[i * pcd->row_step];
      const size_t tgt_i = i/height_increment; // target i

      for(size_t j = 0; j < pcd_width; j += width_increment)
      {
        const uint8_t* data_ptr = &row[j * pcd->point_step];
        const size_t tgt_j = (j/width_increment);
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

  void cloudCB(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg)
  {
    if (sensor_frame == "")
    {
      sensor_frame = msg->header.frame_id;
    }

    scan_.header.stamp = msg->header.stamp;
    scan_.header.frame_id = sensor_frame;
    if(!convert(msg, scan_))
    {
      return;
    }

    pub_scan_->publish(scan_);

    if(debug_cloud)
    {
      sensor_msgs::msg::PointCloud cloud;
      rmcl::convert(scan_, cloud);
      cloud.header.stamp = msg->header.stamp;
      pub_debug_cloud_->publish(cloud);
    }
  }

  // parameters
  std::string sensor_frame = "";
  bool debug_cloud = false;
  size_t height_increment = 1;
  size_t width_increment = 1;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcl_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pub_debug_cloud_;
  rclcpp::Publisher<rmcl_msgs::msg::O1DnStamped>::SharedPtr pub_scan_;
  rmcl_msgs::msg::O1DnStamped scan_;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

};

} // namespace rmcl

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rmcl::Pc2ToO1DnNode)
