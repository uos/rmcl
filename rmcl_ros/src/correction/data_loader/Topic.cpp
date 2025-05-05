#include <rmcl_ros/correction/DataLoader.hpp>
#include <rmcl_ros/correction/data_loader/Topic.hpp>

#include <rmcl_ros/util/conversions.h>

#include <rmagine/util/StopWatch.hpp>

#include <memory>

#include <rmagine/math/statistics.h>
#include <rmagine/math/linalg.h>

#include <rmagine/util/prints.h>



/**
 * 
 * Push before go home:
 * 
 this class was implemented first and its purpose is to just load the lidar data from a specific source (Topics, PLY files, Bag Files).
 However, I was not sure how to do that right, since I have multiple sensors and I am not sure what to trigger in which order.

So I switched to first use this class to do everything. It worked for the "tray" map. After this, I want to move things to other classes 
and reenable the dynamic parameters of ROS 2.

So at first, one must change parameters in this code:
- params_.max_dist (max dist), this is a hard distancte. dynamic thresholding is not ported yet
- n_outer: this is new. the number of outer iterations, aka searching correspondences
- n_inner: this is new. the number of inner iterations, aka optimization steps with fixed correspondences
(total number of itations = n_outer * n_inner)

For GPU: ray casting is cheaper, so its is better to increase the outer iterations and lower the inner iterations
 * 
 * 
 */


namespace rm = rmagine;

namespace rmcl
{

namespace dataloader
{

TopicSourceO1Dn::TopicSourceO1Dn(
  rclcpp::Node::SharedPtr nh, 
  std::string topic_name)
:nh_(nh)
{
  // das kannst du doch keinem zeigen
  tf_buffer_ =
    std::make_shared<tf2_ros::Buffer>(nh_->get_clock());

  // Create the timer interface before call to waitForTransform,
  // to avoid a tf2_ros::CreateTimerInterfaceException exception
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    nh_->get_node_base_interface(),
    nh_->get_node_timers_interface());
  tf_buffer_->setCreateTimerInterface(timer_interface);
  
  tf_listener_ =
    std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  tf_broadcaster_ =
    std::make_shared<tf2_ros::TransformBroadcaster>(*nh_);

  std::cout << "Load data from topic '" << topic_name << "'" << std::endl;

  std::chrono::duration<int> buffer_timeout(1);

  tf_filter_ = std::make_unique<tf2_ros::MessageFilter<rmcl_msgs::msg::O1DnStamped> >(
    data_sub_, *tf_buffer_, odom_frame, 10, nh_->get_node_logging_interface(),
    nh_->get_node_clock_interface(), buffer_timeout);
  
  rclcpp::QoS qos(10); // = rclcpp::SystemDefaultsQoS();
  data_sub_.subscribe(nh_, topic_name, qos.get_rmw_qos_profile()); // delete "get_rmw_..." for rolling
  tf_filter_->registerCallback(&TopicSourceO1Dn::topicCB, this);

  std::cout << "Waiting for message..." << std::endl;
  
  // TODO: not hardcode this
  // std::string map_filename = "/home/amock/rmcl_ws/install/rmcl_examples/share/rmcl_examples/maps/tray.dae";


  // TODO: move this outside the sensor. Otherwise we'd load multiple embree maps
  std::string map_filename = nh_->get_parameter("map_file").as_string();

  std::cout << map_filename << std::endl;

  rm::EmbreeMapPtr map = rm::import_embree_map(map_filename);
  sim_ = std::make_shared<rm::O1DnSimulatorEmbree>(map);

  Tom.setIdentity();

  pose_sub_ = nh_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/initialpose", 10, std::bind(&TopicSourceO1Dn::poseCB, this, std::placeholders::_1));
}

void TopicSourceO1Dn::setMap(rm::EmbreeMapPtr map)
{
  sim_->setMap(map);
}

void TopicSourceO1Dn::poseCB(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  // rm::Transform
  std::cout << "Initial pose guess recievied." << std::endl;
  // TODO: transform pose
  rm::Transform Tbm_est;
  convert(msg->pose.pose, Tbm_est);
  Tom = Tbm_est * ~Tbo; // o -> b -> m
}

void TopicSourceO1Dn::fetchTF()
{
  std::string sensor_frame = "velodyne";
  std::string base_frame = "base_footprint";
  std::string odom_frame = "odom";
  // std::string map_frame = "map";

  // figure out current transform chain.

  try {
    geometry_msgs::msg::TransformStamped T_sensor_base;
    T_sensor_base = tf_buffer_->lookupTransform(base_frame, sensor_frame, stamp_);
    Tsb_stamp = T_sensor_base.header.stamp;
    convert(T_sensor_base.transform, Tsb);
  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN(nh_->get_logger(), "%s", ex.what());
    RCLCPP_WARN_STREAM(nh_->get_logger(), "Source (Sensor): " << sensor_frame << ", Target (Base): " << base_frame);
    return;
  }

  try {
    geometry_msgs::msg::TransformStamped T_base_odom;
    T_base_odom = tf_buffer_->lookupTransform(odom_frame, base_frame, stamp_);
    Tbo_stamp = T_base_odom.header.stamp;
    convert(T_base_odom.transform, Tbo);

  } catch (tf2::TransformException& ex) {
    // std::cout << "Range sensor data is newer than odom! This not too bad. Try to get latest stamp" << std::endl;
    RCLCPP_WARN(nh_->get_logger(), "%s", ex.what());
    RCLCPP_WARN_STREAM(nh_->get_logger(), "Source (Base): " << base_frame << ", Target (Odom): " << odom_frame);
    return;
  }

  std::cout << "TRANSFORM CHAIN COMPLETE!" << std::endl;

  std::cout << "Time Errors:" << std::endl;
  std::cout << "- Tbo - data header: " << (Tbo_stamp - stamp_).seconds() * 1000.0 << "ms" << std::endl; 

  params_.max_dist = 0.5;
}

void TopicSourceO1Dn::topicCB(const rmcl_msgs::msg::O1DnStamped::SharedPtr msg)
{
  stamp_ = msg->header.stamp;

  // Part 1: transfrom sensor and filter input data

  fetchTF();

  rm::StopWatch sw;

  std::cout << "Got scan!!!" << std::endl;

  rclcpp::Time msg_time = msg->header.stamp;
  rclcpp::Duration conversion_time = nh_->get_clock()->now() - msg_time;

  std::cout << "Time lost due to rmcl msg conversion + tf sync: " << conversion_time.seconds() * 1000.0 << "ms" << std::endl;

  // copy to internal representation

  /////
  // sensor model

  std::cout << "INFO: " << msg->o1dn.info.range_min << ", " << msg->o1dn.info.range_max << std::endl;
  rmcl::convert(msg->o1dn.info, sensor_model_);
  sim_->setModel(sensor_model_);
  
  ////
  // data: TODOs: 
  // - use input mask values
  // - use input normals
  size_t n_old_measurements = dataset_.points.size();
  size_t n_new_measurements = msg->o1dn.data.ranges.size();
  if(n_new_measurements > n_old_measurements)
  {
    // need to resize buffers
    std::cout << "Need to resize buffers: " << n_old_measurements << " -> " << n_new_measurements << std::endl;
    dataset_.points.resize(n_new_measurements);
    dataset_.mask.resize(n_new_measurements);
    for(size_t i=n_old_measurements; i<n_new_measurements; i++)
    {
      dataset_.mask[i] = 1;
    }

    rm::resize_memory_bundle<rm::RAM>(simulation_buffers_, sensor_model_.getHeight(), sensor_model_.getWidth(), 1);
  }

  // do we have to set everything to zero?

  sw();
  // fill data
  for(unsigned int vid = 0; vid < sensor_model_.getHeight(); vid++)
  {
    for(unsigned int hid = 0; hid < sensor_model_.getWidth(); hid++)
    {
      const unsigned int loc_id = sensor_model_.getBufferId(vid, hid);
      const float real_range = msg->o1dn.data.ranges[loc_id];
      const rm::Vector3f real_point = sensor_model_.getDirection(vid, hid) * real_range;
      dataset_.points[loc_id] = real_point;

      if(real_range < sensor_model_.range.min || real_range > sensor_model_.range.max)
      {
        // out of range
        dataset_.mask[loc_id] = 0;
      } else {
        dataset_.mask[loc_id] = 1;
      }
    }
  }
  double el = sw();
  
  std::cout << "Fill data (" << n_new_measurements << ") in " << el * 1000.0 << "ms" << std::endl;
  const rm::PointCloudView_<rm::RAM> cloud_dataset = rm::watch(dataset_);
  
  // read Tom -> first Tbm estimation
  rm::Transform Tbm_est = Tom * Tbo;

  // outer loop, find correspondences
  for(size_t i = 0; i<n_outer_; i++)
  {
    // find model correspondences
    findCorrespondences(Tbm_est);
    const rm::PointCloudView_<rm::RAM> cloud_model = {
      .points = simulation_buffers_.points,
      .mask = simulation_buffers_.hits,
      .normals = simulation_buffers_.normals
    };
    
    // inner loop, minimize
    rm::Transform Tdelta = rm::Transform::Identity();
    for(size_t j=0; j<n_inner_; j++)
    {
      // Solve, Umeyama
      const rm::CrossStatistics stats = rm::statistics_p2l(Tdelta, cloud_dataset, cloud_model, params_);
      const rm::Transform Tpre_next = rm::umeyama_transform(stats);
      Tdelta = Tdelta * Tpre_next;
    }
    
    // correct pose
    Tbm_est = Tbm_est * Tdelta;
  }
 
  // write Tom
  Tom = Tbm_est * ~Tbo; // recover Tom: o -> b -> m
  Tom_stamp = stamp_;
  
  { // broadcast transform
    geometry_msgs::msg::TransformStamped T_odom_map;
    T_odom_map.header.stamp = Tom_stamp;
    T_odom_map.header.frame_id = map_frame;
    T_odom_map.child_frame_id = odom_frame;
    convert(Tom, T_odom_map.transform);
    tf_broadcaster_->sendTransform(T_odom_map);
  }

}

void TopicSourceO1Dn::findCorrespondences(const rm::Transform Tbm_est)
{
  sim_->setTsb(Tsb);
  sim_->simulate(Tbm_est, simulation_buffers_);
}

} // namespace dataloader

} // namespace rmcl