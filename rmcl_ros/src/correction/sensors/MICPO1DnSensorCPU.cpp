#include <rmcl_ros/correction/MICPSensor.hpp>
#include <rmcl_ros/correction/sensors/MICPO1DnSensorCPU.hpp>

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

MICPO1DnSensorCPU::MICPO1DnSensorCPU(
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
  tf_filter_->registerCallback(&MICPO1DnSensorCPU::topicCB, this);

  std::cout << "Waiting for message..." << std::endl;

  Tom.setIdentity();

  correspondences_.reset();
}

void MICPO1DnSensorCPU::unpackMessage(
  const rmcl_msgs::msg::O1DnStamped::SharedPtr msg)
{
  /////
  // sensor model
  // std::cout << "INFO: " << msg->o1dn.info.range_min << ", " << msg->o1dn.info.range_max << std::endl;
  rmcl::convert(msg->o1dn.info, sensor_model_);
  
  ////
  // data: TODOs: 
  // - use input mask values
  // - use input normals
  size_t n_old_measurements = correspondences_->dataset.points.size();
  size_t n_new_measurements = msg->o1dn.data.ranges.size();
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
      const float real_range = msg->o1dn.data.ranges[loc_id];
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

void MICPO1DnSensorCPU::topicCB(
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

  if(auto model_setter = std::dynamic_pointer_cast<
    rm::ModelSetter<rm::O1DnModel> >(correspondences_))
  {
    // RCC required sensor model
    model_setter->setModel(sensor_model_);
  }
  
  el = sw();
  
  std::cout << "- Unpack Message & fill data (" 
    << correspondences_->dataset.points.size() << "): " << el * 1000.0 << "ms" << std::endl;
  
  { // print conversion & sync delay
    rclcpp::Time msg_time = msg->header.stamp;
    rclcpp::Duration conversion_time = nh_->get_clock()->now() - msg_time;
    std::cout << "- Time lost due to rmcl msg conversion + tf sync: " << conversion_time.seconds() * 1000.0 << "ms" << std::endl;
  }

  correspondences_->outdated = true;
  first_message_received = true;

  data_correction_mutex_.unlock();
  on_data_received(this);
}

} // namespace rmcl