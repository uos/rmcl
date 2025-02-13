#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <rmcl_msgs/msg/scan_stamped.hpp>

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

class Pcl2ToScanNode : public rclcpp::Node
{
public:
  explicit Pcl2ToScanNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
  :rclcpp::Node("pcl2_to_scan_node", rclcpp::NodeOptions(options)
    .allow_undeclared_parameters(true)
    .automatically_declare_parameters_from_overrides(true))
  {
    fetchParameters();

    pub_scan_ = this->create_publisher<rmcl_msgs::msg::ScanStamped>(
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
    
    rmcl_msgs::msg::ScanInfo &scanner_model = scan_.scan.info;

    if (!this->get_parameter("model.phi_min", scanner_model.phi_min))
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "When specifying auto_detect_phi to false you have to provide model.phi_min");
        return;
    }
    if (!this->get_parameter("model.phi_inc", scanner_model.phi_inc))
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "When specifying auto_detect_phi to false you have to provide model.phi_max");
        return;
    }

    if (!this->get_parameter("model.theta_min", scanner_model.theta_min))
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "When specifying auto_detect_phi to false you have to provide model.phi_min");
        return;
    }
    if (!this->get_parameter("model.theta_inc", scanner_model.theta_inc))
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "When specifying auto_detect_phi to false you have to provide model.phi_max");
        return;
    }

    if (!this->get_parameter("model.range_min", scanner_model.range_min))
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "When specifying auto_detect_phi to false you have to provide model.phi_min");
        return;
    }
    if (!this->get_parameter("model.range_max", scanner_model.range_max))
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "When specifying auto_detect_phi to false you have to provide model.phi_max");
        return;
    }

    int phi_n_tmp, theta_n_tmp;
    if (!this->get_parameter("model.phi_n", phi_n_tmp))
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "When specifying auto_detect_phi to false you have to provide model/phi_min");
        return;
    }
    if (!this->get_parameter("model.theta_n", theta_n_tmp))
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "When specifying auto_detect_phi to false you have to provide model/phi_max");
        return;
    }
    scanner_model.phi_n = phi_n_tmp;
    scanner_model.theta_n = theta_n_tmp;

    if(!this->get_parameter("debug_cloud", debug_cloud))
    {
        debug_cloud = false;
    }
  }

  void initScanArray()
  {
    fillEmpty(scan_.scan);
  }

  bool convert(
      const sensor_msgs::msg::PointCloud2::ConstSharedPtr& pcl,
      rmcl_msgs::msg::ScanStamped& scan) const
  {
    rm::Transform T = rm::Transform::Identity();

    if (pcl->header.frame_id != sensor_frame)
    {
      // TODO: get transform
      geometry_msgs::msg::TransformStamped Tros;

      try
      {
        Tros = tf_buffer_->lookupTransform(sensor_frame, pcl->header.frame_id,
                                            pcl->header.stamp);
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

    fillEmpty(scan.scan);

    sensor_msgs::msg::PointField field_x;
    sensor_msgs::msg::PointField field_y;
    sensor_msgs::msg::PointField field_z;

    for (size_t i = 0; i < pcl->fields.size(); i++)
    {
      if (pcl->fields[i].name == "x")
      {
        field_x = pcl->fields[i];
      }
      if (pcl->fields[i].name == "y")
      {
        field_y = pcl->fields[i];
      }
      if (pcl->fields[i].name == "z")
      {
        field_z = pcl->fields[i];
      }
    }

    rm::SphericalModel model;
    rmcl::convert(scan.scan.info, model);

    for (size_t i = 0; i < pcl->width * pcl->height; i++)
    {
      const uint8_t *data_ptr = &pcl->data[i * pcl->point_step];

      // rmagine::Vector point;
      float x, y, z;

      if (field_x.datatype == sensor_msgs::msg::PointField::FLOAT32)
      {
        // Float
        x = *reinterpret_cast<const float *>(data_ptr + field_x.offset);
        y = *reinterpret_cast<const float *>(data_ptr + field_y.offset);
        z = *reinterpret_cast<const float *>(data_ptr + field_z.offset);
      }
      else if (field_x.datatype == sensor_msgs::msg::PointField::FLOAT64)
      {
        // Double
        x = *reinterpret_cast<const double *>(data_ptr + field_x.offset);
        y = *reinterpret_cast<const double *>(data_ptr + field_y.offset);
        z = *reinterpret_cast<const double *>(data_ptr + field_z.offset);
      }
      else
      {
        throw std::runtime_error("Field X has unknown DataType. Check Topic of pcl");
      }

      if(!std::isnan(x) && !std::isnan(y) && !std::isnan(z))
      {
        rm::Vector ps_s = rm::Vector{x, y, z};
        rm::Vector ps = T * ps_s;

        float range_est = ps.l2norm();
        float theta_est = atan2(ps.y, ps.x); // horizontal
        float phi_est = atan2(ps.z, range_est); // vertical
        
        int phi_id = ((phi_est - model.phi.min) / model.phi.inc) + 0.5;
        int theta_id = ((theta_est - model.theta.min) / model.theta.inc) + 0.5;

        if(phi_id >= 0 && phi_id < (int)model.phi.size
            && theta_id >= 0 && theta_id < (int)model.theta.size)
        {
          if(model.range.inside(range_est))
          {
            unsigned int p_id = model.getBufferId(phi_id, theta_id);
            scan.scan.data.ranges[p_id] = range_est;
          }
        } else {
          // std::cout << "- out scanner matrix" << std::endl;
        }
      }
    }

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

    if (debug_cloud)
    {
      sensor_msgs::msg::PointCloud cloud;
      rmcl::convert(scan_, cloud);
      cloud.header.stamp = msg->header.stamp;
      pub_debug_cloud_->publish(cloud);
    }
  }

  std::string sensor_frame = "";
  bool debug_cloud = false;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcl_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pub_debug_cloud_;
  rclcpp::Publisher<rmcl_msgs::msg::ScanStamped>::SharedPtr pub_scan_;
  rmcl_msgs::msg::ScanStamped scan_;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

};

} // namespace rmcl


#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rmcl::Pcl2ToScanNode)
