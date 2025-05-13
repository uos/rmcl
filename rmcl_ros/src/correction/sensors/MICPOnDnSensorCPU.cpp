#include "rmcl_ros/correction/sensors/MICPOnDnSensorCPU.hpp"

#include <rmcl_ros/util/conversions.h>

#include <rmagine/util/StopWatch.hpp>

#include <memory>
#include <chrono>

#include <rmagine/math/statistics.h>
#include <rmagine/math/linalg.h>

#include <rmagine/util/prints.h>

#include <rmcl_ros/util/ros_helper.h>


using namespace std::chrono_literals;

namespace rm = rmagine;

namespace rmcl
{

MICPOnDnSensorCPU::MICPOnDnSensorCPU(
  rclcpp::Node::SharedPtr nh)
:MICPSensor_<rmagine::RAM>(nh)
{
  Tom.setIdentity();
  correspondences_.reset();

  // check parameter
}

void MICPOnDnSensorCPU::connectToTopic(const std::string& topic_name)
{
  std::chrono::duration<int> buffer_timeout(1);

  tf_filter_ = std::make_unique<tf2_ros::MessageFilter<rmcl_msgs::msg::OnDnStamped> >(
    data_sub_, *tf_buffer_, odom_frame, 10, nh_->get_node_logging_interface(),
    nh_->get_node_clock_interface(), buffer_timeout);

  rclcpp::QoS qos(10); // = rclcpp::SystemDefaultsQoS();
  data_sub_.subscribe(nh_, topic_name, qos.get_rmw_qos_profile()); // delete "get_rmw_..." for rolling
  tf_filter_->registerCallback(&MICPOnDnSensorCPU::updateMsg, this);

  RCLCPP_INFO_STREAM(nh_->get_logger(), "[" << name << "Waiting for message from topic '" << topic_name << "'...");
}

void MICPOnDnSensorCPU::getDataFromParameters()
{
  // rmcl::get_parameter(nh_, "~model.width");
  
  std::cout << "Try to load model" << std::endl;

  const ParamTree<rclcpp::Parameter>::SharedPtr sensor_param_tree
    = get_parameter_tree(nh_, "~");

  const ParamTree<rclcpp::Parameter>::SharedPtr sensor_model_params = sensor_param_tree->at("model");
  sensor_model_.width  = sensor_model_params->at("width")->data->as_int();
  sensor_model_.height = sensor_model_params->at("height")->data->as_int();
  sensor_model_.range.min = sensor_model_params->at("range_min")->data->as_double();
  sensor_model_.range.max = sensor_model_params->at("range_max")->data->as_double();
  sensor_model_.origs.resize(sensor_model_.width * sensor_model_.height);
  sensor_model_.dirs.resize(sensor_model_.width * sensor_model_.height);

  const std::vector<double> origs_data = sensor_model_params->at("origs")->data->as_double_array();
  for(size_t i=0; i<origs_data.size() / 3; i++)
  {
    sensor_model_.origs[i].x = origs_data[i * 3 + 0];
    sensor_model_.origs[i].y = origs_data[i * 3 + 1];
    sensor_model_.origs[i].z = origs_data[i * 3 + 2];
  }

  const std::vector<double> dirs_data = sensor_model_params->at("dirs")->data->as_double_array();
  for(size_t i=0; i<dirs_data.size() / 3; i++)
  {
    sensor_model_.dirs[i].x = dirs_data[i * 3 + 0];
    sensor_model_.dirs[i].y = dirs_data[i * 3 + 1];
    sensor_model_.dirs[i].z = dirs_data[i * 3 + 2];
  }

  if(dirs_data.size() != origs_data.size())
  {
    RCLCPP_ERROR_STREAM(nh_->get_logger(), "OnDn must have same sized dirs and origs!");
    throw std::runtime_error("OnDn must have same sized dirs and origs!");
  }

  const ParamTree<rclcpp::Parameter>::SharedPtr sensor_data_params = sensor_param_tree->at("data");

  const std::vector<double> data_ranges = sensor_data_params->at("ranges")->data->as_double_array();

  if(auto model_setter = std::dynamic_pointer_cast<
    rm::ModelSetter<rm::OnDnModel> >(correspondences_))
  {
    // RCC required sensor model
    model_setter->setModel(sensor_model_);
  }

  size_t n_old_measurements = correspondences_->dataset.points.size();
  size_t n_new_measurements = data_ranges.size();
  if(n_new_measurements > n_old_measurements)
  {
    // need to resize buffers
    correspondences_->dataset.points.resize(n_new_measurements);
    correspondences_->dataset.mask.resize(n_new_measurements);
    for(size_t i=n_old_measurements; i<n_new_measurements; i++)
    {
      correspondences_->dataset.mask[i] = 1;
    }
  }

  total_dataset_measurements = n_new_measurements;
  valid_dataset_measurements = n_new_measurements;


  for(unsigned int vid = 0; vid < sensor_model_.getHeight(); vid++)
  {
    for(unsigned int hid = 0; hid < sensor_model_.getWidth(); hid++)
    {
      const unsigned int loc_id = sensor_model_.getBufferId(vid, hid);
      const float real_range = data_ranges[loc_id];
      const rm::Vector3f real_point = sensor_model_.getDirection(vid, hid) * real_range;
      correspondences_->dataset.points[loc_id] = real_point;
    }
  }

  correspondences_->outdated = true;
  first_message_received = true;

  correspondences_->setTsb(rm::Transform::Identity());

  static_dataset = true;
  dataset_stamp_ = nh_->now();

  on_data_received(this);
}

void MICPOnDnSensorCPU::updateMsg(
  const rmcl_msgs::msg::OnDnStamped::SharedPtr msg)
{
  rm::StopWatch sw;
  const rclcpp::Time msg_time = msg->header.stamp;
  const double diff_now_msg = (nh_->get_clock()->now() - msg_time).seconds();

  if(fabs(diff_now_msg) > 0.1)
  {
    RCLCPP_WARN_STREAM(nh_->get_logger(), "[" << name << "::topicCB] WARNING: (now - input msg's stamp) is more than 100 ms apart (" << diff_now_msg * 1000.0 << " ms). It is likely that control algorithms will not work as expected.");
  }

  sw();
  data_correction_mutex_.lock();
  const double el_mutex_lock = sw();


  sw();
  // Part 1: transfrom sensor and filter input data
  dataset_stamp_ = msg->header.stamp;
  sensor_frame = msg->header.frame_id;
  fetchTF();
  correspondences_->setTsb(Tsb);
  const double el_fetch_tf = sw();

  const double diff_odom_msg = (Tbo_stamp - dataset_stamp_).seconds();

  // copy to internal representation
  // fill sensor_model_ and initialize copy data to dataset
  sw();
  unpackMessage(msg);
  if(auto model_setter = std::dynamic_pointer_cast<
    rm::ModelSetter<rm::OnDnModel> >(correspondences_))
  {
    // RCC required sensor model
    model_setter->setModel(sensor_model_);
  }
  const double el_unpack_msg = sw();

  correspondences_->outdated = true;
  first_message_received = true;

  data_correction_mutex_.unlock();

  { // print stats
    RCLCPP_DEBUG_STREAM(nh_->get_logger(), "[" << name << "::topicCB] MICPOnDnSensorCPU Timings:");
    RCLCPP_DEBUG_STREAM(nh_->get_logger(), "[" << name << "::topicCB] - (Now - msg stamp) = " << diff_now_msg * 1000.0 << " ms");
    RCLCPP_DEBUG_STREAM(nh_->get_logger(), "[" << name << "::topicCB] - (Odom - msg stamp) = " << diff_odom_msg * 1000.0 << " ms");
    RCLCPP_DEBUG_STREAM(nh_->get_logger(), "[" << name << "::topicCB] - Lock mutex: " << el_mutex_lock * 1000.0 << " ms");
    RCLCPP_DEBUG_STREAM(nh_->get_logger(), "[" << name << "::topicCB] - Fetch TF: " << el_fetch_tf * 1000.0 << " ms");
    RCLCPP_DEBUG_STREAM(nh_->get_logger(), "[" << name << "::topicCB] - Unpack message (" << correspondences_->dataset.points.size() << "): " << el_unpack_msg * 1000.0 << " ms");
  }

  on_data_received(this);
}


void MICPOnDnSensorCPU::unpackMessage(
  const rmcl_msgs::msg::OnDnStamped::SharedPtr msg)
{
  /////
  // sensor model
  rmcl::convert(msg->ondn.info, sensor_model_);
  
  ////
  // data: TODOs: 
  // - use input mask values
  // - use input normals
  size_t n_old_measurements = correspondences_->dataset.points.size();
  size_t n_new_measurements = msg->ondn.data.ranges.size();
  if(n_new_measurements > n_old_measurements)
  {
    // need to resize buffers
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
      const float real_range = msg->ondn.data.ranges[loc_id];
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