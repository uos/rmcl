#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

// Rmagine deps
#include <rmagine/map/EmbreeMap.hpp>
#include <rmagine/util/StopWatch.hpp>
#include <rmagine/math/math.h>
#include <rmagine/util/prints.h>
#include <rmagine/simulation/SphereSimulatorEmbree.hpp>
#include <rmagine/simulation/SimulationResults.hpp>

// RCML msgs
#include <rmcl_msgs/msg/o1_dn_stamped.hpp>

// RMCL code
#include <rmcl_ros/util/conversions.h>
#include <rmcl_ros/util/scan_operations.h>
#include <rmcl_ros/util/ros_helper.h>


#include <chrono>
#include <memory>
#include <omp.h>
#include <thread>
#include <mutex>

namespace rm = rmagine;

namespace rmcl
{

class ScanMapSegmentationEmbreeNode 
: public rclcpp::Node
{
public:
  explicit ScanMapSegmentationEmbreeNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
  :rclcpp::Node(
    "scan_map_segmentation_embree_node",
    rclcpp::NodeOptions(options)
      .allow_undeclared_parameters(true)
      .automatically_declare_parameters_from_overrides(true)
  )
  {
    std::cout << "ScanMapSegmentationEmbreeNode started" << std::endl;

    std::string meshfile;
    if (!this->get_parameter("map_file", meshfile))
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "'map_file' is required!");
        throw std::runtime_error("ScanMapSegmentationEmbreeNode::ScanMapSegmentationEmbreeNode() - 'map_file' is required");
    }

    map_frame_ = rmcl::get_parameter(this, "map_frame", "map");

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
      std::bind(&O1DnMapSegmentationEmbreeNode::reconfigureCallback, this, std::placeholders::_1)
    );
    
    rm::EmbreeMapPtr map = rm::import_embree_map(meshfile);
    scan_sim_ = std::make_shared<rm::SphereSimulatorEmbree>(map);
    scan_sim_->setTsb(rm::Transform::Identity());

    tf_buffer_ =
      std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    sub_scan_ = this->create_subscription<rmcl_msgs::msg::O1DnStamped>(
      "scan", 10, 
      [=](const rmcl_msgs::msg::O1DnStamped::ConstSharedPtr& msg) -> void
      { 
        scanCB(msg);
      });

    pub_outlier_scan_ = this->create_publisher<sensor_msgs::msg::PointCloud>(
        "outlier_scan", 10);
    pub_outlier_map_ = this->create_publisher<sensor_msgs::msg::PointCloud>(
        "outlier_map", 10);
  }

  rcl_interfaces::msg::SetParametersResult reconfigureCallback(const std::vector<rclcpp::Parameter>& params)
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

  void scanCB(const rmcl_msgs::msg::O1DnStamped::ConstSharedPtr& msg) const
  {
    geometry_msgs::msg::TransformStamped T_sensor_map;
    
    try{
      T_sensor_map = tf_buffer_->lookupTransform(
        map_frame_,
        msg->header.frame_id,
        msg->header.stamp,
        rclcpp::Duration(1, 0)
      );
    }
    catch (tf2::TransformException &ex) {
      rclcpp::Clock clock = *get_clock();
      RCLCPP_WARN_THROTTLE(
        get_logger(), clock, 5000,
        "Could not transform from '%s' to '%s': %s",
        msg->header.frame_id.c_str(), map_frame_.c_str(), ex.what()
      );
      return;
    }

    rm::Memory<rm::Transform, rm::RAM> T(1);
    // T[0].t.x = T_sensor_map.transform.translation.x;
    convert(T_sensor_map.transform, T[0]);

    // Memory<Transform, VRAM_CUDA> T_ = T;

    rm::O1DnModel model;
    convert(msg->o1dn.info, model);
    scan_sim_->setModel(model);

    using ResultT = rm::Bundle<
      rm::Ranges<rm::RAM>,
      rm::Normals<rm::RAM>
    >;

    ResultT res = scan_sim_->simulate<ResultT>(T);

    rm::MemoryView<float, rm::RAM> ranges = res.ranges;
    rm::MemoryView<rm::Vector, rm::RAM> normals = res.normals;

    // float total_error = 0.0;
    // float dev = 0.0;

    sensor_msgs::msg::PointCloud cloud_outlier_scan;
    cloud_outlier_scan.header.stamp = msg->header.stamp;
    cloud_outlier_scan.header.frame_id = msg->header.frame_id;

    sensor_msgs::msg::PointCloud cloud_outlier_map;
    cloud_outlier_map.header.stamp = msg->header.stamp;
    cloud_outlier_map.header.frame_id = msg->header.frame_id;


    // if this doesnt work: the ranges/dirs of the original scan must be ordered differently
    for(size_t vid = 0; vid < model.getHeight(); vid++)
    {
      for(size_t hid = 0; hid < model.getWidth(); hid++)
      {
        const size_t bid = model.getBufferId(vid, hid);

        const float range_real = msg->scan.data.ranges[bid];
        const float range_sim = ranges[bid];

        const bool range_real_valid = model.range.inside(range_real);
        const bool range_sim_valid = model.range.inside(range_sim);

        if(range_real_valid)
        {
          rm::Vector preal_s = model.getDirection(vid, hid) * range_real + model.getOrigin(vid, hid);

          if(range_sim_valid)
          {
            rm::Vector pint_s = model.getDirection(vid, hid) * range_sim;
            rm::Vector nint_s = normals[bid];
            nint_s.normalizeInplace();

            float signed_plane_dist = (preal_s - pint_s).dot(nint_s);
            const rm::Vector pmesh_s = preal_s + nint_s * signed_plane_dist;
            const float plane_distance = (pmesh_s - preal_s).l2norm();

            if(range_real < range_sim)
            {
              // something is in front of surface
              if( plane_distance > min_dist_outlier_scan_ )
              {
                geometry_msgs::msg::Point32 p_ros;
                p_ros.x = preal_s.x;
                p_ros.y = preal_s.y;
                p_ros.z = preal_s.z;
                cloud_outlier_scan.points.push_back(p_ros);
              }
            } else {
              // ray cutted the surface
              if( plane_distance > min_dist_outlier_map_ )
              {
                geometry_msgs::msg::Point32 p_ros;
                p_ros.x = pint_s.x;
                p_ros.y = pint_s.y;
                p_ros.z = pint_s.z;
                cloud_outlier_map.points.push_back(p_ros);
              }
            }
              
          } else {
              // point in real scan but not in simulated
              geometry_msgs::msg::Point32 p_ros;
              p_ros.x = preal_s.x;
              p_ros.y = preal_s.y;
              p_ros.z = preal_s.z;
              cloud_outlier_scan.points.push_back(p_ros);
          }
        } else {
          if(range_sim_valid)
          {
            // sim hits surface but real not: map could be wrong
            rm::Vector pint_s = model.getDirection(vid, hid) * range_sim + model.getOrigin(vid, hid);
            geometry_msgs::msg::Point32 p_ros;
            p_ros.x = pint_s.x;
            p_ros.y = pint_s.y;
            p_ros.z = pint_s.z;
            cloud_outlier_map.points.push_back(p_ros);
          } else {
            // both sim and real does not hit the map
          }
        }
      }
    }

    pub_outlier_scan_->publish(cloud_outlier_scan);
    pub_outlier_map_->publish(cloud_outlier_map);
}

private:

  float min_dist_outlier_scan_;
  float min_dist_outlier_map_;
  std::string map_frame_;

  rm::SphereSimulatorEmbreePtr scan_sim_;
  
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  
  rclcpp::Subscription<rmcl_msgs::msg::O1DnStamped>::SharedPtr sub_scan_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pub_outlier_scan_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pub_outlier_map_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
};

} // namespace rmcl


#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rmcl::ScanMapSegmentationEmbreeNode)
