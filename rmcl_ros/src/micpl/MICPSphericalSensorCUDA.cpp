#include "rmcl_ros/micpl/MICPSphericalSensorCUDA.hpp"

#include <rmcl_ros/util/conversions.h>
#include <rmcl_ros/util/ros_helper.h>

#include <rmagine/util/StopWatch.hpp>

#include <memory>
#include <chrono>

#include <rmagine/util/prints.h>

using namespace std::chrono_literals;

namespace rm = rmagine;

namespace rmcl
{

MICPSphericalSensorCUDA::MICPSphericalSensorCUDA(
  rclcpp::Node::SharedPtr nh)
:Base(nh)
{
  Tom.setIdentity();
  correspondences_.reset();
}

void MICPSphericalSensorCUDA::connectToTopic(const std::string& topic_name)
{
  static_dataset = false;

  std::chrono::duration<int> buffer_timeout(1);

  tf_filter_ = std::make_unique<tf2_ros::MessageFilter<rmcl_msgs::msg::ScanStamped> >(
    data_sub_, *tf_buffer_, odom_frame, 10, nh_->get_node_logging_interface(),
    nh_->get_node_clock_interface(), buffer_timeout);
  
  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = cb_group_;
  rclcpp::QoS qos(10); // = rclcpp::SystemDefaultsQoS();
  data_sub_.subscribe(nh_, topic_name, qos.get_rmw_qos_profile(), sub_options); // delete "get_rmw_..." for rolling

  tf_filter_->registerCallback(&MICPSphericalSensorCUDA::updateMsg, this);

  RCLCPP_INFO_STREAM(nh_->get_logger(), "[" << name << "] [MICPSphericalSensorCUDA] Waiting for message from topic '" << topic_name << "'...");
}

void MICPSphericalSensorCUDA::getDataFromParameters()
{
  static_dataset = true;
  
  // fill this:
  rmcl_msgs::msg::ScanStamped::SharedPtr scan_stamped
      = std::make_shared<rmcl_msgs::msg::ScanStamped>();

  const ParamTree<rclcpp::Parameter>::SharedPtr sensor_param_tree
    = get_parameter_tree(nh_, "~");

  // 1. Load Model
  const ParamTree<rclcpp::Parameter>::SharedPtr sensor_model_params 
      = sensor_param_tree->at("model");

  if(!convert(sensor_model_params, scan_stamped->scan.info))
  {
    // could parse data from parameters
    throw std::runtime_error("Could not load OnDn model from parameters!");
    return;
  }

  // 2. Load Data
  const ParamTree<rclcpp::Parameter>::SharedPtr sensor_data_params 
      = sensor_param_tree->at("data");

  if(!convert(sensor_data_params, scan_stamped->scan.data))
  {
    // could parse data from parameters
    throw std::runtime_error("Could not load O1Dn data from parameters!");
    return;
  }

  if(sensor_data_params->exists("frame"))
  {
    sensor_frame = sensor_data_params->at("frame")->data->as_string();
  }

  scan_stamped->header.frame_id = sensor_frame;
  scan_stamped->header.stamp = nh_->now();

  updateMsg(scan_stamped);
}

void MICPSphericalSensorCUDA::updateMsg(
  const rmcl_msgs::msg::ScanStamped::SharedPtr msg)
{
  rm::StopWatch sw;
  const rclcpp::Time now_time = nh_->get_clock()->now();
  const rclcpp::Time msg_time = msg->header.stamp;

  if(now_time.get_clock_type() != msg_time.get_clock_type())
  {
    RCLCPP_WARN_STREAM(nh_->get_logger(), "[" << name << "::topicCB] WARNING - STAMP MISMATCH: (now - input msg's stamp) have different clock types: " << now_time.get_clock_type());
    return;
  }

  // const double diff_now_msg = (now_time - msg_time).seconds();
  
  double diff_now_msg;

  try {
    diff_now_msg = (now_time - msg_time).seconds();
  } catch(const std::runtime_error& ex) {
    RCLCPP_WARN_STREAM(nh_->get_logger(), "[" << name << "::topicCB] WARNING - STAMP MISMATCH: (now - input msg's stamp) have different clock type");
    return;
  }

  if(fabs(diff_now_msg) > 0.5)
  {
    RCLCPP_WARN_STREAM(nh_->get_logger(), "[" << name << "::topicCB] WARNING - NETWORK DELAY: (now - input msg's stamp) is more far apart (" << diff_now_msg * 1000.0 << " ms). It is likely that control algorithms will not work as expected.");
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

  // const double diff_odom_msg = (Tbo_stamp - dataset_stamp_).seconds();

  double diff_odom_msg;
  try {
    diff_odom_msg = (Tbo_stamp - dataset_stamp_).seconds();
  } catch(const std::runtime_error& ex) {
    RCLCPP_WARN_STREAM(nh_->get_logger(), "[" << name << "::topicCB] WARNING - STAMP MISMATCH: (now - input msg's stamp) have different clock type");
    data_correction_mutex_.unlock();
    return;
  }

  // copy to internal representation
  // fill sensor_model_ and initialize copy data to dataset
  sw();
  unpackMessage(msg);
  if(auto model_setter = std::dynamic_pointer_cast<
    rm::ModelSetter<rm::SphericalModel> >(correspondences_))
  {
    // RCC required sensor model
    model_setter->setModel(sensor_model_);
  }
  const double el_unpack_msg = sw();

  correspondences_->outdated = true;
  first_message_received = true;
  
  data_correction_mutex_.unlock();

  { // print conversion & sync delay
    RCLCPP_DEBUG_STREAM(nh_->get_logger(), "[" << name << "::topicCB] MICPSphericalSensorCUDA Timings:");
    RCLCPP_DEBUG_STREAM(nh_->get_logger(), "[" << name << "::topicCB] - (Now - msg stamp) = " << diff_now_msg * 1000.0 << " ms");
    RCLCPP_DEBUG_STREAM(nh_->get_logger(), "[" << name << "::topicCB] - (Odom - msg stamp) = " << diff_odom_msg * 1000.0 << " ms");
    RCLCPP_DEBUG_STREAM(nh_->get_logger(), "[" << name << "::topicCB] - Lock mutex: " << el_mutex_lock * 1000.0 << " ms");
    RCLCPP_DEBUG_STREAM(nh_->get_logger(), "[" << name << "::topicCB] - Fetch TF: " << el_fetch_tf * 1000.0 << " ms");
    RCLCPP_DEBUG_STREAM(nh_->get_logger(), "[" << name << "::topicCB] - Unpack message (" << correspondences_->dataset.points.size() << "): " << el_unpack_msg * 1000.0 << " ms");
  }

  if(!static_dataset)
  {
    on_data_received(this);
  }
}

void MICPSphericalSensorCUDA::unpackMessage(
  const rmcl_msgs::msg::ScanStamped::SharedPtr msg)
{
  /////
  // sensor model
  // std::cout << "INFO: " << msg->scan.info.range_min << ", " << msg->scan.info.range_max << std::endl;
  rmcl::convert(msg->scan.info, sensor_model_);
  
  ////
  // data: TODOs: 
  // - use input mask values
  // - use input normals
  size_t n_old_measurements = correspondences_->dataset.points.size();
  size_t n_new_measurements = msg->scan.data.ranges.size();
  if(n_new_measurements > n_old_measurements)
  {
    // need to resize buffers
    correspondences_->dataset.points.resize(n_new_measurements);
    correspondences_->dataset.mask.resize(n_new_measurements);
    dataset_cpu_.points.resize(n_new_measurements);
    dataset_cpu_.mask.resize(n_new_measurements);

    for(size_t i=n_old_measurements; i<n_new_measurements; i++)
    {
      dataset_cpu_.mask[i] = 1;
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
      const float real_range = msg->scan.data.ranges[loc_id];
      const rm::Vector3f real_point = sensor_model_.getDirection(vid, hid) * real_range
                                    + sensor_model_.getOrigin(vid, hid);;
      dataset_cpu_.points[loc_id] = real_point;

      if(real_range < sensor_model_.range.min || real_range > sensor_model_.range.max)
      {
        // out of range
        dataset_cpu_.mask[loc_id] = 0;
      } else {
        dataset_cpu_.mask[loc_id] = 1;
        valid_dataset_measurements++;
      }
    }
  }
  
  // upload
  correspondences_->dataset.points = dataset_cpu_.points;
  correspondences_->dataset.mask = dataset_cpu_.mask;

  dataset_stamp_ = msg->header.stamp;
}

} // namespace rmcl