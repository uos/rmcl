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

#include <rmcl_ros/correction/sensors/MICPO1DnSensorCPU.hpp>
#include <rmcl_ros/correction/correspondences/RCCEmbree.hpp>
#include <rmcl_ros/correction/correspondences/CPCEmbree.hpp>

#include <rmcl_ros/correction/sensors/MICPO1DnSensorCUDA.hpp>
#include <rmcl_ros/correction/correspondences/RCCOptix.hpp>

using namespace std::chrono_literals;

namespace rm = rmagine;

namespace rmcl
{

MICPLocalizationNode::MICPLocalizationNode(const rclcpp::NodeOptions& options)
:rclcpp::Node("micp_localization_node", rclcpp::NodeOptions(options)
    .allow_undeclared_parameters(true)
    .automatically_declare_parameters_from_overrides(true))
{
  Tom = rmagine::Transform::Identity();
  
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
  map_optix_ = rm::import_optix_map(map_filename_);

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
        sensors_vec_.push_back(sensor);

      } else {
        std::string sensor_name = elem.second->name;
        std::cout << "Couldn't load sensor: '" << sensor_name << "'" << std::endl;
      }
    }
  } else {
    RCLCPP_ERROR(get_logger(), "ERROR: NO SENSORS SPECIFIED!");
    throw std::runtime_error("ERROR: NO SENSORS SPECIFIED!");
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

  // last_correction_stamp = this->now();
}

void MICPLocalizationNode::poseCB(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  std::cout << "POSE RECEIVED!" << std::endl;
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
    elem.second->setTom(Tom); // Tbm = Tom * Tbo
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
  
  // std::cout << "Param tree:" << std::endl;
  // sensor_param_tree->print();
  
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
      
      if(corr_backend == "embree")
      {
        auto sensor_cpu = std::make_shared<MICPO1DnSensorCPU>(nh_sensor, topic_name);

        auto rcc_embree = std::make_shared<RCCEmbreeO1Dn>(map_embree_);
        rcc_embree->params.max_dist = 1.0;
        auto cpc_embree = std::make_shared<CPCEmbree>(map_embree_);
        cpc_embree->params.max_dist = 1.0;

        sensor_cpu->correspondences_ = rcc_embree;
        sensor_cpu->on_data_received = std::bind(&MICPLocalizationNode::sensorDataReceived, this, std::placeholders::_1);
      
        sensor = sensor_cpu;
      }

      if(corr_backend == "optix")
      {
        auto sensor_gpu = std::make_shared<MICPO1DnSensorCUDA>(nh_sensor, topic_name);

        auto rcc_optix = std::make_shared<RCCOptixO1Dn>(map_optix_);
        rcc_optix->params.max_dist = 1.0;

        sensor_gpu->correspondences_ = rcc_optix;
        sensor_gpu->on_data_received = std::bind(&MICPLocalizationNode::sensorDataReceived, this, std::placeholders::_1);
      
        sensor = sensor_gpu;
      }
    }
  }

  return sensor;
}

void MICPLocalizationNode::sensorDataReceived(
  const MICPSensorBase* sensor)
{
  std::cout << sensor->name << " received data!" << std::endl;
  data_stamp_latest_ = sensor->dataset_stamp_;
}

void MICPLocalizationNode::correct()
{
  size_t outer_iter = 3;
  for(size_t i=0; i<outer_iter; i++)
  {
    correctOnce();
  }
}

void MICPLocalizationNode::correctOnce()
{
  // #pragma omp parallel for
  for(auto sensor : sensors_vec_)
  {
    // dont change the state of this sensor
    sensor->mutex().lock();
  }

  // #pragma omp parallel for
  for(auto sensor : sensors_vec_)
  {
    // only set current transform from odom to map
    // Tbo and Tsb are fetched synchron to the arriving sensor data
    sensor->setTom(Tom);
    sensor->findCorrespondences();
  }

  rm::Transform T_onew_oold = rm::Transform::Identity();

  
  for(size_t i=0; i<optimization_iterations_; i++)
  {
    // std::cout << "Correct! " << correction_counter << ", " << i << std::endl;
    rm::CrossStatistics Cmerged_o = rm::CrossStatistics::Identity();

    bool outdated = false;

    // #pragma omp parallel for
    for(const auto sensor : sensors_vec_)
    {
      if(sensor->correspondencesOutdated())
      {
        std::cout << "Coresspondences outdated!" << std::endl;
        outdated = true;
        continue;
      }
      
      // transform delta from odom frame to base frame, at time of respective sensor
      const rm::Transform T_bnew_bold = ~sensor->Tbo * T_onew_oold * sensor->Tbo;

      const rm::CrossStatistics Cs_b 
        = sensor->computeCrossStatistics(T_bnew_bold);
      
      const rm::CrossStatistics Cs_o = sensor->Tbo * Cs_b;

      #pragma omp critical
      {
        Cmerged_o += Cs_o; // optimal merge in odom frame
      }
    }

    if(outdated)
    {
      break;
    }

    // Cmerged_o -> T_onew_oold
    const rm::Transform T_onew_oold_inner = rm::umeyama_transform(Cmerged_o);

    // update T_onew_oold: 
    // transform from new odom frame to old odom frame
    // this is only virtual (a trick). we dont't want to change the odom frame in the end (instead the odom to map transform)
    T_onew_oold = T_onew_oold * T_onew_oold_inner;
  }

  correction_counter++;
  // we want T_onew_map, we have Tom which is T_oold_map
  const rm::Transform T_onew_map = Tom * T_onew_oold;
  Tom = T_onew_map; // store Tom

  // #pragma omp parallel for
  for(auto sensor : sensors_vec_)
  {
    sensor->mutex().unlock();
  }
}

void MICPLocalizationNode::broadcastTransform()
{
  geometry_msgs::msg::TransformStamped T_odom_map;
  
  // T_odom_map.header.stamp = data_stamp_latest_; // correct time. lags behind
  T_odom_map.header.stamp = this->now(); // better viz. includes the odom errors for a short period of time

  T_odom_map.header.frame_id = map_frame_;
  T_odom_map.child_frame_id = odom_frame_;
  convert(Tom, T_odom_map.transform);
  tf_broadcaster_->sendTransform(T_odom_map);
}

void MICPLocalizationNode::correctionLoop()
{
  bool all_first_message_received = false;
  while(rclcpp::ok() && !stop_correction_thread_ && !all_first_message_received)
  {
    all_first_message_received = true;
    for(auto sensor : sensors_vec_)
    {
      all_first_message_received &= sensor->first_message_received;
    }
  }

  std::cout << "Every sensor recevied first message! Starting correction" << std::endl;

  while(rclcpp::ok() && !stop_correction_thread_)
  {
    // RCLCPP_INFO_STREAM(this->get_logger(), "Latest data received vs now: " << data_stamp_latest_.seconds() << " vs " << this->get_clock()->now().seconds());
    
    // TODO: what if we have different computing units?
    // RTX have a smaller bottleneck vs Embree
    // We could do this in parallel
    correctOnce();
    broadcastTransform();
  }
  
}

} // namespace rmcl

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rmcl::MICPLocalizationNode)
