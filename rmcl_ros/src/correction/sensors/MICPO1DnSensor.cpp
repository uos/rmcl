#include <rmcl_ros/correction/MICPSensor.hpp>
#include <rmcl_ros/correction/sensors/MICPO1DnSensor.hpp>

#include <rmcl_ros/util/conversions.h>

#include <rmagine/util/StopWatch.hpp>

#include <memory>

#include <rmagine/math/statistics.h>
#include <rmagine/math/linalg.h>

#include <rmagine/util/prints.h>



namespace rm = rmagine;

namespace rmcl
{

MICPO1DnSensor::MICPO1DnSensor(
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
  tf_filter_->registerCallback(&MICPO1DnSensor::topicCB, this);

  std::cout << "Waiting for message..." << std::endl;
  
  // TODO: move this outside the sensor. Otherwise we'd load multiple embree maps
  std::string map_filename = nh_->get_parameter("map_file").as_string();

  std::cout << map_filename << std::endl;

  rm::EmbreeMapPtr map = rm::import_embree_map(map_filename);
  sim_ = std::make_shared<rm::O1DnSimulatorEmbree>(map);

  Tom.setIdentity();

  pose_sub_ = nh_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/initialpose", 10, std::bind(&MICPO1DnSensor::poseCB, this, std::placeholders::_1));
}

void MICPO1DnSensor::setMap(rm::EmbreeMapPtr map)
{
  sim_->setMap(map);
}

void MICPO1DnSensor::poseCB(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  // rm::Transform
  std::cout << "Initial pose guess recieved." << std::endl;
  // TODO: transform pose
  rm::Transform Tbm_est;
  convert(msg->pose.pose, Tbm_est);
  Tom = Tbm_est * ~Tbo; // o -> b -> m
}

void MICPO1DnSensor::fetchTF()
{
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

void MICPO1DnSensor::topicCB(const rmcl_msgs::msg::O1DnStamped::SharedPtr msg)
{
  stamp_ = msg->header.stamp;

  sensor_frame = msg->header.frame_id;

  // Part 1: transfrom sensor and filter input data

  fetchTF();
  sim_->setTsb(Tsb);

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



  // correction!

  

  // read Tom -> first Tbm estimation
  rm::Transform Tbm_est = Tom * Tbo;

  // outer loop, find correspondences
  for(size_t i = 0; i<n_outer_; i++)
  {
    // per sensor

    // find model correspondences
    const rm::PointCloudView_<rm::RAM> cloud_model = findCorrespondences(Tbm_est);

    // inner loop, minimize
    // rm::Transform Tdelta_b = rm::Transform::Identity();

    // this is what we want to optimize: 
    // find a transformation from a new base frame to the old base frame that optimizes the alignment
    
    // bnew --- T_bnew_bold --> bold    : base frame (shared frame of all robot sensors)
    //  ^                        ^
    //  |                        |
    // Tsb                      Tsb
    //  |                        |
    // snew --- T_snew_sold --> sold    : sensor frames
    // 
    // Fig 1: How to transform delta transforms

    rm::Transform T_bnew_bold = rm::Transform::Identity();
    for(size_t j=0; j<n_inner_; j++)
    {
      // from snew to bnew to bold to sold -> snew to sold
      rm::Transform T_snew_sold = ~Tsb * T_bnew_bold * Tsb;

      const rm::CrossStatistics stats_s = rm::statistics_p2l(T_snew_sold, cloud_dataset, cloud_model, params_);
      
      // delta transform of inner loop
      const rm::Transform T_sinner_snew = rm::umeyama_transform(stats_s);
      
      T_snew_sold = T_snew_sold * T_sinner_snew;
      
      // recover actual bnew to bold (go a path through Fig. 1)
      // from bnew to to snew to sold to bold -> bnew to bold
      T_bnew_bold = Tsb * T_snew_sold * ~Tsb;
    }

    // update estimate
    Tbm_est = Tbm_est * T_bnew_bold;
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

rm::PointCloudView_<rm::RAM> MICPO1DnSensor::findCorrespondences(const rm::Transform Tbm_est)
{
  sim_->simulate(Tbm_est, simulation_buffers_);

  const rm::PointCloudView_<rm::RAM> cloud_model = {
    .points = simulation_buffers_.points,
    .mask = simulation_buffers_.hits,
    .normals = simulation_buffers_.normals
  };

  return cloud_model;
}

} // namespace rmcl