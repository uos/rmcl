#include "rmcl_ros/nodes/filter/scan_map_segmentation_embree.hpp"


// Rmagine deps
#include <rmagine/map/EmbreeMap.hpp>
#include <rmagine/util/StopWatch.hpp>
#include <rmagine/math/math.h>
#include <rmagine/util/prints.h>
#include <rmagine/simulation/SimulationResults.hpp>
#include <rmagine/simulation/SphereSimulatorEmbree.hpp>

// RMCL code
#include <rmcl_ros/util/conversions.h>
#include <rmcl_ros/util/scan_operations.h>
#include <rmcl_ros/util/ros_helper.h>


#include <sensor_msgs/point_cloud_conversion.hpp>

#include <memory>
#include <chrono>
#include <omp.h>
#include <thread>
#include <mutex>


namespace rm = rmagine;

namespace rmcl
{

ScanMapSegmentationEmbreeNode::ScanMapSegmentationEmbreeNode(
  const rclcpp::NodeOptions& options)
:MapSegmentationNode("scan_map_segmentation_embree_node", options)
{
  std::cout << "ScanMapSegmentationEmbreeNode started" << std::endl;

  rm::EmbreeMapPtr map = rm::import_embree_map(map_file_);
  scan_sim_ = std::make_shared<rm::SphereSimulatorEmbree>(map);
  scan_sim_->setTsb(rm::Transform::Identity());

  sub_scan_ = this->create_subscription<rmcl_msgs::msg::ScanStamped>(
    "scan", 10, 
    [=](const rmcl_msgs::msg::ScanStamped::ConstSharedPtr& msg) -> void
    { 
      scanCB(msg);
    });
}

void ScanMapSegmentationEmbreeNode::scanCB(
  const rmcl_msgs::msg::ScanStamped::ConstSharedPtr& msg) const
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

  rm::Transform T;
  // T[0].t.x = T_sensor_map.transform.translation.x;
  convert(T_sensor_map.transform, T);

  // Memory<Transform, VRAM_CUDA> T_ = T;

  rm::O1DnModel model;
  convert(msg->o1dn.info, model);
  scan_sim_->setModel(model);

  using ResultT = rm::Bundle<
    rm::Ranges<rm::RAM>,
    rm::Normals<rm::RAM>
  >;

  ResultT res = scan_sim_->simulate<ResultT>(T);

  const rm::MemoryView<float, rm::RAM> ranges = res.ranges;
  const rm::MemoryView<rm::Vector, rm::RAM> normals = res.normals;

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

  sensor_msgs::msg::PointCloud2 cloud_outlier_scan2;
  sensor_msgs::msg::PointCloud2 cloud_outlier_map2;
  sensor_msgs::convertPointCloudToPointCloud2(cloud_outlier_scan, cloud_outlier_scan2);
  sensor_msgs::convertPointCloudToPointCloud2(cloud_outlier_map, cloud_outlier_map2);

  pub_outlier_scan_->publish(cloud_outlier_scan2);
  pub_outlier_map_->publish(cloud_outlier_map2);
}

} // namespace rmcl

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rmcl::ScanMapSegmentationEmbreeNode)
