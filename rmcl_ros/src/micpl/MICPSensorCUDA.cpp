#include "rmcl_ros/micpl/MICPSensorCUDA.hpp"


namespace rm = rmagine;

namespace rmcl
{

MICPSensorCUDA::MICPSensorCUDA(rclcpp::Node::SharedPtr nh)
: Base(nh)
{
  
}

void MICPSensorCUDA::drawCorrespondences()
{
  if(!enable_visualizations)
  {
    return;
  }
  
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = sensor_frame;
  
  if(static_dataset)
  {
    marker.header.stamp = Tbo_stamp;
  } else {
    marker.header.stamp = dataset_stamp_;
  }

  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.type = visualization_msgs::msg::Marker::LINE_LIST;

  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;

  std_msgs::msg::ColorRGBA color_start;
  color_start.r = 0.0;
  color_start.g = 0.0;
  color_start.b = 0.0;
  color_start.a = 0.8;

  std_msgs::msg::ColorRGBA color_end;
  color_end.r = 1.0;
  color_end.g = 1.0;
  color_end.b = 1.0;
  color_end.a = 0.8;

  marker.pose.position.x = 0.0;
  marker.pose.position.y = 0.0;
  marker.pose.position.z = 0.0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  marker.scale.x = 0.05;
  marker.scale.y = 0.05;
  marker.scale.z = 0.05;

  // Download model
  // This is the reason why you should disable visualizations
  const auto model_cpu = rm::transfer<rm::RAM>(correspondences_->modelView());

  const rm::UmeyamaReductionConstraints corr_params
    = correspondences_->params;

  // P2L
  for(size_t i=0; i<model_cpu.points.size(); i++)
  {
    if(model_cpu.mask[i] > 0 && dataset_cpu_.mask[i] > 0)
    {
      const rm::Vector Di = dataset_cpu_.points[i]; // read
      const rm::Vector Ii = model_cpu.points[i]; // read
      const rm::Vector Ni = model_cpu.normals[i];

      const float signed_plane_dist = (Ii - Di).dot(Ni);

      if(fabs(signed_plane_dist) < corr_params.max_dist)
      {
        const rm::Vector Mi = Di + Ni * signed_plane_dist;
        geometry_msgs::msg::Point p1, p2;

        p1.x = Di.x;
        p1.y = Di.y;
        p1.z = Di.z;
        p2.x = Mi.x;
        p2.y = Mi.y;
        p2.z = Mi.z;

        marker.points.push_back(p1);
        marker.points.push_back(p2);

        marker.colors.push_back(color_start);
        marker.colors.push_back(color_end);
      }
    }
  }

  correspondence_viz_pub_->publish(marker);
}

} // namespace rmcl