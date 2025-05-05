#ifndef RMCL_ROS_CORRECTION_DATA_LOADER_TOPIC_HPP
#define RMCL_ROS_CORRECTION_DATA_LOADER_TOPIC_HPP

#include <rclcpp/rclcpp.hpp>
#include <rmcl_ros/correction/DataLoader.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <rmcl_msgs/msg/o1_dn_stamped.hpp>

#include <rmagine/types/sensor_models.h>
#include <rmagine/simulation/O1DnSimulatorEmbree.hpp>

#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include "tf2_ros/transform_broadcaster.h"
#include <tf2_ros/buffer.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/create_timer_ros.h>

#include <message_filters/subscriber.h>


#include <rmagine/math/statistics.h>
#include <rmagine/math/linalg.h>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>



namespace rmcl
{


class RegistrationMinimizer
{
public:
  virtual rmagine::Transform minimize(
    const rmagine::Transform Tinit, 
    const rmagine::PointCloudView_<rmagine::RAM> dataset,
    const rmagine::PointCloudView_<rmagine::RAM> model) = 0;
};


class UmeyamaMinimizer
: public RegistrationMinimizer
{
public:

  UmeyamaMinimizer(rmagine::UmeyamaReductionConstraints params)
  :RegistrationMinimizer()
  ,params_(params)
  {
    // construct
  }

  virtual rmagine::Transform minimize(
    const rmagine::Transform Tpre, 
    const rmagine::PointCloudView_<rmagine::RAM> dataset,
    const rmagine::PointCloudView_<rmagine::RAM> model) override
  {
    namespace rm = rmagine;
    const rm::CrossStatistics stats = rm::statistics_p2l(Tpre, dataset, model, params_);
    const rm::Transform Tpre_next = rm::umeyama_transform(stats);
    const rm::Transform Tpre_opti = Tpre * Tpre_next;
    return Tpre_opti;
  }

protected:
  rmagine::UmeyamaReductionConstraints params_;
};

namespace dataloader
{

class TopicSourceO1Dn
: public DataLoader_<rmagine::RAM>
{
public:
  TopicSourceO1Dn(
    rclcpp::Node::SharedPtr nh,
    std::string topic_name);
    
  void poseCB(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  
  /**
   * Fetch TF chain. except for odom->map (this is estimated or has to be provided from extern)
   */
  void fetchTF();

  void topicCB(const rmcl_msgs::msg::O1DnStamped::SharedPtr msg);

  void setMap(rmagine::EmbreeMapPtr map);

  rmagine::PointCloudView_<rmagine::RAM> findCorrespondences(const rmagine::Transform Tbm_est);

  

  // transform chain from sensor -> base -> odom -> map
  // keep this up to date
  rmagine::Transform Tsb;
  rclcpp::Time Tsb_stamp;
  rmagine::Transform Tbo;
  rclcpp::Time Tbo_stamp;
  rmagine::Transform Tom;
  rclcpp::Time Tom_stamp;

  std::string map_frame = "map";
  std::string odom_frame = "odom";
  std::string base_frame = "base_footprint";
  std::string sensor_frame = "velodyne";

  size_t n_outer_ = 2;
  size_t n_inner_ = 10;
  rmagine::UmeyamaReductionConstraints params_;

private:
  rclcpp::Node::SharedPtr nh_;

  rmagine::O1DnModel sensor_model_;
  rmagine::O1DnSimulatorEmbreePtr sim_;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  message_filters::Subscriber<rmcl_msgs::msg::O1DnStamped> data_sub_;

  std::unique_ptr<tf2_ros::MessageFilter<rmcl_msgs::msg::O1DnStamped> > tf_filter_;

  // simulation buffer
  rmagine::Bundle<
    rmagine::Points<rmagine::RAM>, 
    rmagine::Normals<rmagine::RAM>,
    rmagine::Hits<rmagine::RAM>
  > simulation_buffers_;

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;

};

} // namespace dataloader

} // namespace rmcl

#endif // RMCL_ROS_CORRECTION_DATA_LOADER_TOPIC_HPP