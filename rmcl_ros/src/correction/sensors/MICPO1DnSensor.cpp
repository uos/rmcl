#include <rmcl_ros/correction/MICPSensor.hpp>
#include <rmcl_ros/correction/sensors/MICPO1DnSensor.hpp>

#include <rmcl_ros/util/conversions.h>

#include <rmagine/util/StopWatch.hpp>

#include <memory>
#include <chrono>

#include <rmagine/math/statistics.h>
#include <rmagine/math/linalg.h>

#include <rmagine/util/prints.h>

using namespace std::chrono_literals;

namespace rm = rmagine;

namespace rmcl
{

MICPO1DnSensor::MICPO1DnSensor(
  rclcpp::Node::SharedPtr nh, 
  std::string topic_name)
:MICPSensor_<rmagine::RAM>(nh)
{
  std::cout << "Load data from topic '" << topic_name << "'" << std::endl;

  std::chrono::duration<int> buffer_timeout(1);

  tf_filter_ = std::make_unique<tf2_ros::MessageFilter<rmcl_msgs::msg::O1DnStamped> >(
    data_sub_, *tf_buffer_, odom_frame, 10, nh_->get_node_logging_interface(),
    nh_->get_node_clock_interface(), buffer_timeout);
  
  rclcpp::QoS qos(10); // = rclcpp::SystemDefaultsQoS();
  data_sub_.subscribe(nh_, topic_name, qos.get_rmw_qos_profile()); // delete "get_rmw_..." for rolling
  tf_filter_->registerCallback(&MICPO1DnSensor::topicCB, this);

  std::cout << "Waiting for message..." << std::endl;

  Tom.setIdentity();

  params_.max_dist = 1.0;

  correspondences_.reset();
}

void MICPO1DnSensor::unpackMessage(
  const rmcl_msgs::msg::O1DnStamped::SharedPtr msg)
{
  /////
  // sensor model
  std::cout << "INFO: " << msg->o1dn.info.range_min << ", " << msg->o1dn.info.range_max << std::endl;
  rmcl::convert(msg->o1dn.info, sensor_model_);
  
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
  }

  // sw();
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
  // double el = sw();
  dataset_stamp_ = msg->header.stamp;
}

void MICPO1DnSensor::topicCB(
  const rmcl_msgs::msg::O1DnStamped::SharedPtr msg)
{
  // std::lock_guard<std::mutex> guard(data_correction_mutex_);
  data_correction_mutex_.lock();

  rm::StopWatch sw;
  double el;

  dataset_stamp_ = msg->header.stamp;

  sensor_frame = msg->header.frame_id;

  // Part 1: transfrom sensor and filter input data
  fetchTF();
  correspondences_->setTsb(Tsb);

  // copy to internal representation

  // fill sensor_model_ and initialize copy data to dataset
  sw();
  unpackMessage(msg);
  if(auto rcc = std::dynamic_pointer_cast<RCCEmbreeO1Dn>(correspondences_))
  {
    // RCC required sensor model
    rcc->setModel(sensor_model_);
  }
  
  el = sw();
  
  std::cout << "Unpack Message & fill data (" 
    << dataset_.points.size() << "): " << el * 1000.0 << "ms" << std::endl;
  
  
  { // print conversion & sync delay
    rclcpp::Time msg_time = msg->header.stamp;
    rclcpp::Duration conversion_time = nh_->get_clock()->now() - msg_time;
    std::cout << "Time lost due to rmcl msg conversion + tf sync: " << conversion_time.seconds() * 1000.0 << "ms" << std::endl;
  }

  correspondences_->outdated = true;

  first_message_received = true;

  data_correction_mutex_.unlock();


  on_data_received(this);

  // return; // everything from here has to be move to an upper level

  // correct();
  // broadcastTransform();
}

// void MICPO1DnSensor::correctOnce()
// {
//   std::lock_guard<std::mutex> guard(data_correction_mutex_);

//   if(!first_message_received)
//   {
//     std::cout << "Waiting for first message to arrive..." << std::endl;
//     return;
//   }
  
//   if(!correspondences_)
//   {
//     std::cout << "Correspondences not set!" << std::endl;
//     return;
//   }

//   // per sensor
    
//   // find model correspondences

//   std::cout << "find corr" << std::endl;
//   findCorrespondences();
//   const rm::PointCloudView_<rm::RAM> cloud_dataset = rm::watch(dataset_);
//   const rm::PointCloudView_<rm::RAM> cloud_model = correspondences_->get();

//   // inner loop, minimize

//   // this is what we want to optimize: 
//   // find a transformation from a new base frame to the old base frame that optimizes the alignment
  
//   // bnew --- T_bnew_bold --> bold    : base frame (shared frame of all robot sensors)
//   //  ^                        ^
//   //  |                        |
//   // Tsb                      Tsb
//   //  |                        |
//   // snew --- T_snew_sold --> sold    : sensor frames
//   // 
//   // Fig 1: How to transform delta transforms

//   // THIS IS WORKING VERY GOOD:
//   // rm::Transform T_bnew_bold = rm::Transform::Identity();
//   // for(size_t j=0; j<n_inner_; j++)
//   // {
//   //   const rm::CrossStatistics C_b = computeCrossStatistics(T_bnew_bold);
//   //   const rm::Transform T_binner_bnew = rm::umeyama_transform(C_b);
//   //   T_bnew_bold = T_bnew_bold * T_binner_bnew;
//   // }
//   // // update estimate
//   // Tbm_est = Tbm_est * T_bnew_bold;
//   // setTom(Tbm_est * ~Tbo);

//   // Problem: dataset can change, when new data arrives. I dont want to provide two datasets
//   const rmagine::Transform Tbo = this->Tbo;

//   // because we later want to fuse in odom frame, this is a test for a single sensor:
//   rm::Transform T_onew_oold = rm::Transform::Identity();
//   for(size_t j=0; j<n_inner_; j++)
//   {
//     if(correspondences_->outdated)
//     {
//       std::cout << "WARNING CORRESPONDENCES OUTDATED!" << std::endl;
//       break;
//     }
//     // back to base
//     const rm::Transform T_bnew_bold = ~Tbo * T_onew_oold * Tbo;
//     const rm::CrossStatistics C_b = computeCrossStatistics(T_bnew_bold);
//     // back to odom (where we can fuse other measurements)
//     const rm::CrossStatistics C_o = Tbo * C_b;
    
//     // do this for all sensors, then:
//     rm::CrossStatistics C_merged = rm::CrossStatistics::Identity();
//     C_merged += C_o;

//     const rm::Transform T_oinner_onew = rm::umeyama_transform(C_merged);
//     T_onew_oold = T_onew_oold * T_oinner_onew;
//   }
//   setTom(Tom * T_onew_oold);

//   // { // broadcast transform
//   //   geometry_msgs::msg::TransformStamped T_odom_map;
//   //   // T_odom_map.header.stamp = dataset_stamp_;
//   //   T_odom_map.header.stamp = nh_->now();
//   //   T_odom_map.header.frame_id = map_frame;
//   //   T_odom_map.child_frame_id = odom_frame;
//   //   convert(Tom, T_odom_map.transform);
//   //   tf_broadcaster_->sendTransform(T_odom_map);
//   // }
// }

// void MICPO1DnSensor::correct()
// {
//   if(!first_message_received)
//   {
//     std::cout << "Waiting for first message to arrive..." << std::endl;
//     return;
//   }
//   // just do this for the whole period of time

//   // outer loop, find correspondences
//   for(size_t i = 0; i<n_outer_; i++)
//   {
//     correctOnce();
//   }

//   // write Tom
//   // Tom = Tbm_est * ~Tbo; // recover Tom: o -> b -> m
//   Tom_stamp = dataset_stamp_;
// }

// void MICPO1DnSensor::correctionLoop()
// {
//   while(!first_message_received)
//   {
//     std::cout << "Waiting for first message to arrive..." << std::endl;
//     std::this_thread::sleep_for(10ms);
//   }

//   while(rclcpp::ok() && !stop_correction_thread_)
//   {
//     std::cout << "correctOnce" << std::endl;
//     // for(size_t i = 0; i<n_outer_; i++)
//     // {
//     correctOnce();
//     // }
//     Tom_stamp = dataset_stamp_;
//     broadcastTransform();
//   }

//   // while(rclcpp::ok() && !stop_correction_thread_)
//   // {
//   //   broadcastTransform();
//   //   std::this_thread::sleep_for(100ms);
//   // }
// }

// void MICPO1DnSensor::broadcastTransform()
// {
//   geometry_msgs::msg::TransformStamped T_odom_map;

//   const rclcpp::Time time_now = nh_->now();
//   std::cout << "broadcast transform odom -> map: " << time_now.seconds() << std::endl;

//   // T_odom_map.header.stamp = Tom_stamp;
//   T_odom_map.header.stamp = time_now;
//   T_odom_map.header.frame_id = map_frame;
//   T_odom_map.child_frame_id = odom_frame;
//   convert(Tom, T_odom_map.transform);
//   tf_broadcaster_->sendTransform(T_odom_map);
// }

} // namespace rmcl