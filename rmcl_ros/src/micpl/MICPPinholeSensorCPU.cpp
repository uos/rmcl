#include "rmcl_ros/micpl/MICPPinholeSensorCPU.hpp"

#include <rmcl_ros/micpl/MICPSensor.hpp>
#include <rmcl_ros/util/conversions.h>
#include <rmcl_ros/util/ros_helper.h>

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

MICPPinholeSensorCPU::MICPPinholeSensorCPU(
  rclcpp::Node::SharedPtr nh)
:MICPSensorCPU(nh)
{
  Tom.setIdentity();
  correspondences_.reset();
}

void MICPPinholeSensorCPU::connectToTopic(const std::string& topic_name)
{
  static_dataset = false;

  std::chrono::duration<int> buffer_timeout(1);

  tf_filter_ = std::make_unique<tf2_ros::MessageFilter<rmcl_msgs::msg::DepthStamped> >(
    data_sub_, *tf_buffer_, odom_frame, 10, nh_->get_node_logging_interface(),
    nh_->get_node_clock_interface(), buffer_timeout);
  
  rclcpp::QoS qos(10);
  data_sub_.subscribe(nh_, topic_name, qos.get_rmw_qos_profile()); // delete "get_rmw_..." for rolling
  tf_filter_->registerCallback(&MICPPinholeSensorCPU::updateMsg, this);

  RCLCPP_INFO_STREAM(nh_->get_logger(), "[" << name << "] [MICPPinholeSensorCPU] " << "Waiting for message from topic '" << topic_name << "'...");
}

void MICPPinholeSensorCPU::getDataFromParameters()
{
  static_dataset = true;
  
  // fill this:
  rmcl_msgs::msg::DepthStamped::SharedPtr depth_stamped
      = std::make_shared<rmcl_msgs::msg::DepthStamped>();

  const ParamTree<rclcpp::Parameter>::SharedPtr sensor_param_tree
    = get_parameter_tree(nh_, "~");

  // 1. Load Model
  const ParamTree<rclcpp::Parameter>::SharedPtr sensor_model_params 
      = sensor_param_tree->at("model");

  if(!convert(sensor_model_params, depth_stamped->depth.info))
  {
    // could parse data from parameters
    throw std::runtime_error("Could not load OnDn model from parameters!");
    return;
  }

  // 2. Load Data
  const ParamTree<rclcpp::Parameter>::SharedPtr sensor_data_params 
      = sensor_param_tree->at("data");

  if(!convert(sensor_data_params, depth_stamped->depth.data))
  {
    // could parse data from parameters
    throw std::runtime_error("Could not load O1Dn data from parameters!");
    return;
  }

  if(sensor_data_params->exists("frame"))
  {
    sensor_frame = sensor_data_params->at("frame")->data->as_string();
  }

  depth_stamped->header.frame_id = sensor_frame;
  depth_stamped->header.stamp = nh_->now();

  updateMsg(depth_stamped);
}

void MICPPinholeSensorCPU::updateMsg(
  const rmcl_msgs::msg::DepthStamped::SharedPtr msg)
{
  rm::StopWatch sw;
  const rclcpp::Time msg_time = msg->header.stamp;
  const double diff_now_msg = (nh_->get_clock()->now() - msg_time).seconds();

  if(fabs(diff_now_msg) > 0.1)
  {
    RCLCPP_WARN_STREAM(nh_->get_logger(), "[" << name << "] WARNING - NETWORK DELAY: (now - input msg's stamp) is far apart (" << diff_now_msg * 1000.0 << " ms). It is likely that control algorithms will not work as expected.");
  }
  
  sw();
  data_correction_mutex_.lock();
  const double el_mutex_lock = sw();

  sw();
  // Part 1: transfrom sensor and filter input data
  dataset_stamp_ = msg->header.stamp;
  sensor_frame = msg->header.frame_id;
  fetchTF(dataset_stamp_);
  correspondences_->setTsb(Tsb);
  const double el_fetch_tf = sw();

  const double diff_odom_msg = (Tbo_stamp - dataset_stamp_).seconds();

  // copy to internal representation
  // fill sensor_model_ and initialize copy data to dataset
  sw();
  unpackMessage(msg);

  // TODO: make some kind of PinholeSetter base class that doesnt depend on Embree
  if(auto model_setter = std::dynamic_pointer_cast<
    rm::ModelSetter<rm::PinholeModel> >(correspondences_))
  {
    // RCC required sensor model
    model_setter->setModel(sensor_model_);
  }
  
  const double el_unpack_msg = sw();

  correspondences_->outdated = true;
  first_message_received = true;

  data_correction_mutex_.unlock();

  { // print stats
    RCLCPP_DEBUG_STREAM(nh_->get_logger(), "[" << name << "::topicCB] MICPPinholeSensorCPU Timings:");
    RCLCPP_DEBUG_STREAM(nh_->get_logger(), "[" << name << "::topicCB] - (Now - msg stamp) = " << diff_now_msg * 1000.0 << " ms");
    RCLCPP_DEBUG_STREAM(nh_->get_logger(), "[" << name << "::topicCB] - (Odom - msg stamp) = " << diff_odom_msg * 1000.0 << " ms");
    RCLCPP_DEBUG_STREAM(nh_->get_logger(), "[" << name << "::topicCB] - Lock mutex: " << el_mutex_lock * 1000.0 << " ms");
    RCLCPP_DEBUG_STREAM(nh_->get_logger(), "[" << name << "::topicCB] - Fetch TF: " << el_fetch_tf * 1000.0 << " ms");
    RCLCPP_DEBUG_STREAM(nh_->get_logger(), "[" << name << "::topicCB] - Unpack message (" << correspondences_->dataset.points.size() << "): " << el_unpack_msg * 1000.0 << " ms");
  }

  on_data_received(this);
}

void MICPPinholeSensorCPU::unpackMessage(
  const rmcl_msgs::msg::DepthStamped::SharedPtr msg)
{
  /////
  // sensor model
  // std::cout << "INFO: " << msg->o1dn.info.range_min << ", " << msg->o1dn.info.range_max << std::endl;
  rmcl::convert(msg->depth.info, sensor_model_);
  
  ////
  // data: TODOs: 
  // - use input mask values
  // - use input normals
  size_t n_old_measurements = correspondences_->dataset.points.size();
  size_t n_new_measurements = msg->depth.data.ranges.size();
  if(n_new_measurements > n_old_measurements)
  {
    // need to resize buffers
    // std::cout << "Need to resize buffers: " << n_old_measurements << " -> " << n_new_measurements << std::endl;
    correspondences_->dataset.points.resize(n_new_measurements);
    correspondences_->dataset.mask.resize(n_new_measurements);
    for(size_t i=n_old_measurements; i<n_new_measurements; i++)
    {
      correspondences_->dataset.mask[i] = 1;
    }
  }

  total_dataset_measurements = n_new_measurements;
  valid_dataset_measurements = 0;

  // sw();
  // fill data
  for(unsigned int vid = 0; vid < sensor_model_.getHeight(); vid++)
  {
    for(unsigned int hid = 0; hid < sensor_model_.getWidth(); hid++)
    {
      const unsigned int loc_id = sensor_model_.getBufferId(vid, hid);
      const float real_range = msg->depth.data.ranges[loc_id];
      const rm::Vector3f real_point = sensor_model_.getDirection(vid, hid) * real_range;
      correspondences_->dataset.points[loc_id] = real_point;

      if(real_range < sensor_model_.range.min || real_range > sensor_model_.range.max)
      {
        // out of range
        correspondences_->dataset.mask[loc_id] = 0;
      } else {
        valid_dataset_measurements++;
        correspondences_->dataset.mask[loc_id] = 1;
      }
    }
  }
  
  dataset_stamp_ = msg->header.stamp;
}

} // namespace rmcl