#include <rclcpp/rclcpp.hpp>

#include <rmagine/util/prints.h>
#include <rmagine/util/StopWatch.hpp>
#include <rmagine/math/math.h>
#ifdef RMCL_CUDA
#include <rmagine/math/math.cuh>
#endif // RMCL_CUDA

// #include <rmcl_ros/correction/MICP.hpp>
#include <rmcl_ros/util/conversions.h>
#include <rmcl_ros/util/ros_helper.h>

#include <thread>
#include <mutex>
#include <chrono>
#include <memory>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <rmcl_ros/nodes/micp_localization.hpp>
// #include <rmcl_ros/correction/MICPSensorSphericalEmbree.hpp>

#include <rmcl_ros/correction/sensors/MICPO1DnSensor.hpp>


// #include <exception>

#include <rmcl_ros/correction/DataLoader.hpp>
#include <rmcl_ros/correction/data_loader/Topic.hpp>


// #include <rmcl_ros/correction/RCCEmbree.hpp>

using namespace std::chrono_literals;

namespace rm = rmagine;

namespace rmcl
{

MICPLocalizationNode::MICPLocalizationNode(const rclcpp::NodeOptions& options)
:rclcpp::Node("micp_localization_node", rclcpp::NodeOptions(options)
    .allow_undeclared_parameters(true)
    .automatically_declare_parameters_from_overrides(true))
{
  
  
  std::cout << "MICPLocalizationNode" << std::endl;

  { // set up tf
    tf_buffer_ =
      std::make_shared<tf2_ros::Buffer>(this->get_clock());

    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      this->get_node_base_interface(),
      this->get_node_timers_interface());
    tf_buffer_->setCreateTimerInterface(timer_interface);

    tf_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    tf_broadcaster_ =
      std::make_shared<tf2_ros::TransformBroadcaster>(*this);
  }


  base_frame_ = rmcl::get_parameter(this, "base_frame", "base_link");
  map_frame_ = rmcl::get_parameter(this, "map_frame", "map");

  odom_frame_ = rmcl::get_parameter(this, "odom_frame", "");
  use_odom_frame_ = (odom_frame_ != "");

  map_filename_ = rmcl::get_parameter(this, "map_file", "");
  if(map_filename_ == "")
  {
    RCLCPP_ERROR(get_logger(), "User must provide ~map_file");
    throw std::runtime_error("User must provide ~map_file");
  }

  std::cout << "MAP FILE: " << map_filename_ << std::endl;

  map_embree_ = rm::import_embree_map(map_filename_);

  // loading general micp config

  // loading sensors from parameter tree
  std::map<std::string, rclcpp::Parameter> sensors_param;
  if(this->get_parameters("sensors", sensors_param))
  {
    std::cout << std::endl;
    std::cout << "-------------------------" << std::endl;
    std::cout << "     --- SENSORS ---     " << std::endl;
    std::cout << "-------------------------" << std::endl;

    ParamTree<rclcpp::Parameter>::SharedPtr sensors_param_tree 
      = get_parameter_tree(this, "sensors");
    for(auto elem : *sensors_param_tree)
    {
      auto sensor = loadSensor(elem.second);
      if(sensor)
      {
        std::cout << "Loaded:  " << sensor->name << std::endl;
        sensors_[sensor->name] = sensor;

      } else {
        std::string sensor_name = elem.second->name;
        std::cout << "Couldn't load sensor: '" << sensor_name << "'" << std::endl;
      }
    }
  } else {
    RCLCPP_ERROR(get_logger(), "ERROR: NO SENSORS SPECIFIED!");
    throw std::runtime_error("UERROR: NO SENSORS SPECIFIED!");
  }

  std::cout << "MICP load params - done. Valid Sensors: " << sensors_.size() << std::endl;

  std::chrono::duration<int> buffer_timeout(1);

  // incoming pose this needs to be synced with tf
  pose_tf_filter_ = std::make_unique<tf2_ros::MessageFilter<geometry_msgs::msg::PoseWithCovarianceStamped> >(
    pose_sub_, *tf_buffer_, odom_frame_, 10,this->get_node_logging_interface(),
    this->get_node_clock_interface(), buffer_timeout);

  rclcpp::QoS qos(10); // = rclcpp::SystemDefaultsQoS();
  pose_sub_.subscribe(this, "/initialpose", qos.get_rmw_qos_profile()); // delete "get_rmw_..." for rolling
  pose_tf_filter_->registerCallback(&MICPLocalizationNode::poseCB, this);

  // pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
  //   "/initialpose", 10, std::bind(&MICPLocalizationNode::poseCB, this, std::placeholders::_1));

  std::cout << "Waiting for pose..." << std::endl;

  // timer. get latest tf

  // correction_timer_ =  this->create_wall_timer(
  //   500ms, std::bind(&MICPLocalizationNode::correction_callback, this));
  stop_correction_thread_ = false;
  correction_thread_ = std::thread([&](){
    correctionLoop();
  });
  // correction_thread_.detach();

}



void MICPLocalizationNode::poseCB(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  rm::Transform Tbo;
  rclcpp::Time Tbo_stamp;

  try {
    geometry_msgs::msg::TransformStamped T_base_odom;
    T_base_odom = tf_buffer_->lookupTransform(odom_frame_, base_frame_, msg->header.stamp);
    Tbo_stamp = T_base_odom.header.stamp;
    convert(T_base_odom.transform, Tbo);

  } catch (tf2::TransformException& ex) {
    // std::cout << "Range sensor data is newer than odom! This not too bad. Try to get latest stamp" << std::endl;
    RCLCPP_WARN(this->get_logger(), "%s", ex.what());
    RCLCPP_WARN_STREAM(this->get_logger(), "Source (Base): " << base_frame_ << ", Target (Odom): " << odom_frame_);
    return;
  }

  // normally the pose is set in map coords
  // just 
  rm::Transform T_pc_m = rm::Transform::Identity();
  if(msg->header.frame_id != map_frame_)
  {
    // search for transform from pose to map
    try {
      geometry_msgs::msg::TransformStamped T_pose_map;
      T_pose_map = tf_buffer_->lookupTransform(map_frame_, msg->header.frame_id, msg->header.stamp);
      convert(T_pose_map.transform, T_pc_m);
    } catch (tf2::TransformException& ex) {
      // std::cout << "Range sensor data is newer than odom! This not too bad. Try to get latest stamp" << std::endl;
      RCLCPP_WARN(this->get_logger(), "%s", ex.what());
      RCLCPP_WARN_STREAM(this->get_logger(), "Source (Pose): " << msg->header.frame_id << ", Target (Map): " << map_frame_);
      return;
    }
  }

  // transform from actual pose (base) to the pose coordinate system
  rm::Transform T_b_pc;
  convert(msg->pose.pose, T_b_pc);
  // figure out what transform from odom to map is

  // transform b -> m == transform b -> pc -> m
  const rm::Transform Tbm = T_pc_m * T_b_pc;
  Tom = Tbm * ~Tbo;
  
  // transform

  // rm::Transform
  std::cout << "Initial pose guess processed." << std::endl;

  // TODO: remove this
  // update sensors
  for(auto elem : sensors_)
  {
    elem.second->setTbm(Tom * Tbm); // Tbm = Tom * Tbo
  }
}

MICPSensorPtr MICPLocalizationNode::loadSensor(
  ParamTree<rclcpp::Parameter>::SharedPtr sensor_params)
{
  MICPSensorPtr sensor;

  std::string sensor_name = sensor_params->name;

  std::cout << "Loading '" << sensor_name << "'" << std::endl;

  rclcpp::Node::SharedPtr nh_sensor = this->create_sub_node("sensors/" + sensor_name);
  
  ParamTree<rclcpp::Parameter>::SharedPtr sensor_param_tree
    = get_parameter_tree(nh_sensor, "~");
  
  std::cout << "Param tree:" << std::endl;
  sensor_param_tree->print();
  
  if(!sensor_param_tree->exists("type"))
  {
    // ERROR!
    throw std::runtime_error("PARAM ERROR");
  }
  
  // fetch parameters that decide which implementation is loaded
  const std::string sensor_type = sensor_param_tree->at("type")->data->as_string();
  std::cout << "- Type: " << sensor_type << std::endl;
  const std::string model_source = sensor_param_tree->at("model_source")->data->as_string();  
  std::cout << "- Model Source: " << model_source << std::endl;
  const std::string data_source = sensor_param_tree->at("data_source")->data->as_string();
  std::cout << "- Data Source: " << data_source << std::endl;

  const std::string corr_backend = sensor_param_tree->at("correspondences")->at("backend")->data->as_string();
  std::cout << "- Correspondence Backend: " << corr_backend << std::endl;

  if(data_source == "topic")
  {
    const std::string topic_name = sensor_param_tree->at("topic")->at("name")->data->as_string();
    const std::string topic_type = sensor_param_tree->at("topic")->at("type")->data->as_string();

    if(topic_type == "rmcl_msgs/msg/O1DnStamped")
    {
      sensor = std::make_shared<MICPO1DnSensor>(nh_sensor, topic_name);
      sensor->name = sensor_name;
      
      if(corr_backend == "embree")
      {
        sensor->correspondences_ = std::make_shared<RCCEmbreeO1Dn>(map_embree_);
        // sensor->on_data_received = std::bind(&MICPLocalizationNode::sensorDataReceived, this, std::placeholders::_1);
      }
    }
  }

  return sensor;
}

void MICPLocalizationNode::sensorDataReceived(
  const MICPSensorBase* sensor)
{
  std::cout << sensor->name << " received data!" << std::endl;

}

void MICPLocalizationNode::correctionLoop()
{

  

  while(rclcpp::ok() && !stop_correction_thread_)
  {
    std::cout << "Correct!" << std::endl;

    size_t outer_iters = 10;
    size_t inner_iters = 2;

    for(size_t i=0; i<outer_iters; i++)
    {
      std::cout << "Find Correspondences!" << std::endl;
      for(auto sensor_elem : sensors_)
      {
        
      }
    }    


    std::this_thread::sleep_for(50ms);
  }
}

} // namespace rmcl

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rmcl::MICPLocalizationNode)
