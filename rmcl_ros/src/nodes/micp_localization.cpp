#include "rmcl_ros/nodes/micp_localization.hpp"

#include <rclcpp/rclcpp.hpp>

#include <rmagine/util/prints.h>
#include <rmagine/util/StopWatch.hpp>

#include <rmagine/math/linalg.h>

#include <rmcl_ros/util/conversions.h>
#include <rmcl_ros/util/ros_helper.h>
#include <rmcl_ros/util/text_colors.h>

#include <thread>
#include <mutex>
#include <chrono>
#include <memory>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>


#include <rmcl_ros/micpl/MICPSphericalSensorCPU.hpp>
#include <rmcl_ros/micpl/MICPPinholeSensorCPU.hpp>
#include <rmcl_ros/micpl/MICPOnDnSensorCPU.hpp>
#include <rmcl_ros/micpl/MICPO1DnSensorCPU.hpp>

#ifdef RMCL_EMBREE
#include <rmcl/registration/RCCEmbree.hpp>
#include <rmcl/registration/CPCEmbree.hpp>
#endif // RMCL_EMBREE

#ifdef RMCL_CUDA
#include <rmcl_ros/micpl/MICPSphericalSensorCUDA.hpp>
#include <rmcl_ros/micpl/MICPPinholeSensorCUDA.hpp>
#include <rmcl_ros/micpl/MICPO1DnSensorCUDA.hpp>
#include <rmcl_ros/micpl/MICPOnDnSensorCUDA.hpp>
#endif // RMCL_CUDA

#ifdef RMCL_OPTIX
#include <rmcl/registration/RCCOptix.hpp>
#endif // RMCL_OPTIX

#include <rmcl_msgs/msg/micp_stats.hpp>
#include <rmcl_msgs/msg/micp_sensor_stats.hpp>

using namespace std::chrono_literals;

namespace rm = rmagine;

namespace rmcl
{

bool check(const rm::Vector& v)
{
  return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

bool check(const rm::Quaternion& q)
{
  return std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z) && std::isfinite(q.w);
}

bool check(const rm::Transform& T)
{
  return check(T.t) && check(T.R);
}

template<typename T>
bool is_valid(T a)
{
  return a == a;
}

template<typename DataT>
void checkStats(rm::CrossStatistics_<DataT> stats)
{
  // check for nans
  if(!is_valid(stats.dataset_mean.x)){throw std::runtime_error("ERROR: NAN");};
  if(!is_valid(stats.dataset_mean.y)){throw std::runtime_error("ERROR: NAN");};
  if(!is_valid(stats.dataset_mean.z)){throw std::runtime_error("ERROR: NAN");};
  
  if(!is_valid(stats.model_mean.x)){throw std::runtime_error("ERROR: NAN");};
  if(!is_valid(stats.model_mean.y)){throw std::runtime_error("ERROR: NAN");};
  if(!is_valid(stats.model_mean.z)){throw std::runtime_error("ERROR: NAN");};

  if(!is_valid(stats.covariance(0,0))){throw std::runtime_error("ERROR: NAN");};
  if(!is_valid(stats.covariance(0,1))){throw std::runtime_error("ERROR: NAN");};
  if(!is_valid(stats.covariance(0,2))){throw std::runtime_error("ERROR: NAN");};

  if(!is_valid(stats.covariance(1,0))){throw std::runtime_error("ERROR: NAN");};
  if(!is_valid(stats.covariance(1,1))){throw std::runtime_error("ERROR: NAN");};
  if(!is_valid(stats.covariance(1,2))){throw std::runtime_error("ERROR: NAN");};

  if(!is_valid(stats.covariance(2,0))){throw std::runtime_error("ERROR: NAN");};
  if(!is_valid(stats.covariance(2,1))){throw std::runtime_error("ERROR: NAN");};
  if(!is_valid(stats.covariance(2,2))){throw std::runtime_error("ERROR: NAN");};
}

MICPLocalizationNode::MICPLocalizationNode(const rclcpp::NodeOptions& options)
:rclcpp::Node("micp_localization_node", rclcpp::NodeOptions(options)
    .allow_undeclared_parameters(true)
    .automatically_declare_parameters_from_overrides(true))
{
  Tom_ = rmagine::Transform::Identity();
  Tbo_latest_ = rmagine::Transform::Identity();
  
  std::cout << "MICPLocalizationNode" << std::endl;

  // 1. Load parameters
  base_frame_ = rmcl::get_parameter(this, "base_frame", "base_link");
  map_frame_ = rmcl::get_parameter(this, "map_frame", "map");
  odom_frame_ = rmcl::get_parameter(this, "odom_frame", "");

  map_filename_ = rmcl::get_parameter(this, "map_file", "");
  if(map_filename_ == "")
  {
    RCLCPP_ERROR(get_logger(), "User must provide ~map_file");
    throw std::runtime_error("User must provide ~map_file");
  }

  disable_correction_ = rmcl::get_parameter(this, "disable_correction", false);
  tf_time_source_ = rmcl::get_parameter(this, "tf_time_source", 0);
  optimization_iterations_ = rmcl::get_parameter(this, "optimization_iterations", 5);
  correction_rate_max_ = rmcl::get_parameter(this, "correction_rate_max", 1000.0);
  int max_cpu_threads = rmcl::get_parameter(this, "max_cpu_threads", 4);
  omp_set_num_threads(max_cpu_threads);

  broadcast_tf_ = rmcl::get_parameter(this, "broadcast_tf", true);
  tf_rate_ = rmcl::get_parameter(this, "tf_rate", 100.0);
  publish_pose_ = rmcl::get_parameter(this, "publish_pose", false);

  pose_noise_ = rmcl::get_parameter(this, "pose_noise", 0.01);
  adaptive_max_dist_ = rmcl::get_parameter(this, "adaptive_max_dist", true);

  std::vector<double> initial_pose_offset = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  initial_pose_offset = rmcl::get_parameter(this, "initial_pose_offset", initial_pose_offset);

  initial_pose_offset_.t.x = initial_pose_offset[0];
  initial_pose_offset_.t.y = initial_pose_offset[1];
  initial_pose_offset_.t.z = initial_pose_offset[2];

  if(initial_pose_offset.size() == 6)
  {
    rm::EulerAngles euler;
    euler.roll = initial_pose_offset[3];
    euler.pitch = initial_pose_offset[4];
    euler.yaw = initial_pose_offset[5];
    initial_pose_offset_.R = euler;
  }
  else if(initial_pose_offset.size() == 7)
  {
    initial_pose_offset_.R.x = initial_pose_offset[3];
    initial_pose_offset_.R.y = initial_pose_offset[4];
    initial_pose_offset_.R.z = initial_pose_offset[5];
    initial_pose_offset_.R.w = initial_pose_offset[6];
  }

  #ifdef RMCL_EMBREE
  map_embree_ = rm::import_embree_map(map_filename_);
  #endif // RMCL_EMBREE
  #ifdef RMCL_OPTIX
  map_optix_ = rm::import_optix_map(map_filename_);
  #endif // RMCL_OPTIX
  // loading general micp config

  const ParamTree<rclcpp::Parameter>::SharedPtr sensors_param_tree 
    = get_parameter_tree(this, "sensors");
  if(sensors_param_tree->size() == 0)
  {
    RCLCPP_ERROR(get_logger(), "ERROR: NO SENSORS SPECIFIED!");
    throw std::runtime_error("ERROR: NO SENSORS SPECIFIED!");
  }

  for(auto elem : *sensors_param_tree)
  {
    auto sensor = loadSensor(elem.second);
    if(sensor)
    {
      sensors_[sensor->name] = sensor;
      sensors_vec_.push_back(sensor);
      if(!sensor->static_dataset)
      {
        num_dynamic_sensors_++;
      }
      sensor->asyncSpin();
    } else {
      std::string sensor_name = elem.second->name;
      std::cout << "Couldn't load sensor: '" << sensor_name << "'" << std::endl;
    }
  }
  
  printSetup();

  std::cout << "MICP load params - done. Valid Sensors: " << sensors_.size() << std::endl;

  // SETUP ROS CONNECTIONS

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

    int num_tries = 0;
    // odom and base frames have to be available!
    while(!tf_buffer_->_frameExists(base_frame_))
    {
      num_tries++;
      RCLCPP_INFO_STREAM(this->get_logger(), "Waiting for '" << base_frame_ << "' frame to become available ... (" << num_tries << ")");
      // this->get_clock()->sleep_for(std::chrono::duration<double>(1.0));
      std::this_thread::sleep_for(std::chrono::duration<double>(1.0));
    }
    RCLCPP_INFO_STREAM(this->get_logger(), "'" << base_frame_ << "' found.");
    std::cout << "Done." << std::endl;

    num_tries = 0;
    while(!tf_buffer_->_frameExists(odom_frame_))
    {
      num_tries++;
      RCLCPP_INFO_STREAM(this->get_logger(), "Waiting for '" << odom_frame_ << "' frame to become available ... (" << num_tries << ")");
      this->get_clock()->sleep_for(std::chrono::duration<double>(1.0));
    }
  }

  // incoming pose this needs to be synced with tf
  stats_publisher_ = this->create_publisher<rmcl_msgs::msg::MICPSensorStats>("~/micpl_stats", 10);

  pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/initialpose", 10, std::bind(&MICPLocalizationNode::poseCB, this, std::placeholders::_1));

  if(publish_pose_)
  {
    Tbm_publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("~/micpl_pose", 10);
  }

  std::cout << "Waiting for pose..." << std::endl;

  // timer. get latest tf
  stop_correction_thread_ = false;
  correction_thread_ = std::thread([&](){
    correctionLoop();
  });
  // correction_thread_.detach();
  
  stop_tf_broadcaster_thread_ = false;
  tf_broadcaster_thread_ = std::thread([&](){
    tfBroadcastLoop();
  });

  // last_correction_stamp = this->now();
}

void MICPLocalizationNode::printSetup()
{
  std::cout << std::endl;
  std::cout << "-------------------------" << std::endl;
  std::cout << "       --- MAP ---       " << std::endl;
  std::cout << "-------------------------" << std::endl;

  rm::AssimpIO io;
  const aiScene* ascene = io.ReadFile(map_filename_, 0);

  std::cout << "- file: " << map_filename_ << std::endl;
  std::cout << "- meshes: " << ascene->mNumMeshes << std::endl;
  for(size_t i=0; i<ascene->mNumMeshes; i++)
  {
    const aiMesh* amesh = ascene->mMeshes[i];
    std::cout << TC_SENSOR << amesh->mName.C_Str() << TC_END << std::endl;
    std::cout << "  - vertices, faces: " << amesh->mNumVertices << ", " << amesh->mNumFaces << std::endl;
  }
  std::cout << "For more infos enter in terminal: " << std::endl;
  std::cout << "$ rmagine_map_info " << map_filename_ << std::endl;
  

  std::cout << std::endl;
  std::cout << "--------------------------" << std::endl;
  std::cout << "     --- BACKENDS ---     " << std::endl;
  std::cout << "--------------------------" << std::endl;

  std::cout << "Available combining units:" << std::endl;
  std::cout << "- " << TC_BACKENDS << "CPU" << TC_END << std::endl;
  #ifdef RMCL_CUDA
  std::cout << "- " << TC_BACKENDS << "GPU" << TC_END << std::endl; 
  #endif // RMCL_CUDA

  std::cout << "Available raytracing backends:" << std::endl;
  #ifdef RMCL_EMBREE
  std::cout << "- " << TC_BACKENDS << "Embree (CPU)" << TC_END << std::endl;
  #endif // RMCL_EMBREE

  #ifdef RMCL_OPTIX
  std::cout << "- " << TC_BACKENDS << "Optix (GPU)" << TC_END << std::endl;
  #endif // RMCL_OPTIX

  std::cout << std::endl;
  std::cout << "-------------------------" << std::endl;
  std::cout << "     --- FRAMES ---      " << std::endl;
  std::cout << "-------------------------" << std::endl;

  std::cout << "- base:\t" << TC_FRAME << base_frame_ << TC_END << std::endl;
  std::cout << "- odom:\t" << TC_FRAME << odom_frame_ << TC_END << std::endl;
  std::cout << "- map:\t" << TC_FRAME << map_frame_ << TC_END << std::endl;
  std::cout << "Estimating: " << TC_FRAME << base_frame_ << TC_END << " -> " << TC_FRAME << map_frame_ << TC_END << std::endl;
  std::cout << "Providing:  " << TC_FRAME << odom_frame_ << TC_END << " -> " << TC_FRAME << map_frame_ << TC_END << std::endl;


  std::cout << std::endl;
  std::cout << "-------------------------" << std::endl;
  std::cout << "     --- SENSORS ---     " << std::endl;
  std::cout << "-------------------------" << std::endl;

  for(const auto& sensor : sensors_vec_)
  {
    const ParamTree<rclcpp::Parameter>::SharedPtr sensor_params 
      = get_parameter_tree(this, "sensors." + sensor->name);

    std::cout << "- " << TC_SENSOR << sensor->name << TC_END << std::endl;

    std::string data_source = sensor_params->at("data_source")->data->as_string();
    std::cout << "  - data:\t" << data_source << std::endl;
    if(data_source == "topic")
    {
      std::cout << "    - topic:\t" << TC_TOPIC << sensor_params->at("topic_name")->data->as_string() << TC_END << std::endl;
      // while(!sensor->first_message_received)
      // {
      //   // Wait
      //   this->get_clock()->sleep_for(std::chrono::duration<float>(0.1));
      // }
    } else if(data_source == "parameters") {
      // TODO: 
    }

    std::cout << "    - frame:\t" << TC_FRAME << sensor->sensor_frame << TC_END << std::endl;
    
    std::cout << "  - model:\t" << TC_MSG << sensor_params->at("model_type")->data->as_string() << TC_END << std::endl;

    const ParamTree<rclcpp::Parameter>::SharedPtr corr_params
      = sensor_params->at("correspondences"); 

    std::cout << "  - correspondences: " << std::endl;
    std::cout << "     - backend: " << TC_BLUE << corr_params->at("backend")->data->as_string() << TC_END << std::endl;
    std::cout << "     - type:    " << TC_MAGENTA << corr_params->at("type")->data->as_string() << TC_END << std::endl;
    std::cout << "     - metric:  " << TC_GREEN << corr_params->at("metric")->data->as_string() << TC_END << std::endl;


  }
}

void MICPLocalizationNode::poseCB(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  RCLCPP_INFO_STREAM(get_logger(), "Initial pose guess received.");
  
  // check if RViz was started wrong

  const rclcpp::Time now_time = this->now();
  const rclcpp::Time msg_time = msg->header.stamp;

  double time_diff;
  try{
    time_diff = (now_time - msg_time).seconds();
  } catch(const std::runtime_error& ex) {
    RCLCPP_WARN_STREAM(this->get_logger(), "Received pose has different time source than MICP-L node (Have you started RViz with the same use_sim_time settings?)");
    return;
  }
  std::cout << "Time Diff (now - pose): " << time_diff << " s" << std::endl;

  // wait for correction loop to finish
  
  // normally the pose is set in map coords
  rm::Transform T_pc_m = rm::Transform::Identity();
  if(msg->header.frame_id != map_frame_)
  {
    try {
      if(tf_buffer_->canTransform(
        map_frame_, msg->header.frame_id, msg->header.stamp, rclcpp::Duration::from_seconds(1.0)))
      {
        // search for transform from pose to map
        const geometry_msgs::msg::TransformStamped T_pose_map 
          = tf_buffer_->lookupTransform(map_frame_, msg->header.frame_id, msg->header.stamp);
        convert(T_pose_map.transform, T_pc_m);
      }
      else
      {
        RCLCPP_WARN_STREAM(this->get_logger(), "[MICPLocalizationNode::poseCB] Timeout. Transform from '" << msg->header.frame_id << "' to '" << map_frame_ << "' not available yet.");
        return;
      }
    } catch (tf2::TransformException& ex) {
      // std::cout << "Range sensor data is newer than odom! This not too bad. Try to get latest stamp" << std::endl;
      RCLCPP_WARN(this->get_logger(), "%s", ex.what());
      RCLCPP_WARN_STREAM(this->get_logger(), "Source (Pose): " << msg->header.frame_id << ", Target (Map): " << map_frame_);
      return;
    }
  }

  rm::Transform Tbo;

  try {
    if(tf_buffer_->canTransform(
      odom_frame_, base_frame_, msg->header.stamp, rclcpp::Duration::from_seconds(1.0)))
    {
      // search for transform from pose to map
      const geometry_msgs::msg::TransformStamped T_base_odom 
        = tf_buffer_->lookupTransform(odom_frame_, base_frame_, msg->header.stamp);
      convert(T_base_odom.transform, Tbo);
    }
    else
    {
      RCLCPP_WARN_STREAM(this->get_logger(), "[MICPLocalizationNode::poseCB] Timeout. Transform from '" << base_frame_ << "' to '" << odom_frame_ << "' not available yet.");
      return;
    }
  } catch (tf2::TransformException& ex) {
    // std::cout << "Range sensor data is newer than odom! This not too bad. Try to get latest stamp" << std::endl;
    RCLCPP_WARN(this->get_logger(), "%s", ex.what());
    RCLCPP_WARN_STREAM(this->get_logger(), "Source (Base): " << base_frame_ << ", Target (Odom): " << odom_frame_);
    return;
  }

  // transform from actual pose (base) to the pose coordinate system
  rm::Transform T_b_pc;
  convert(msg->pose.pose, T_b_pc);
  // figure out what transform from odom to map is

  // transform b -> m == transform b -> pc -> m
  const rm::Transform Tbm = T_pc_m * T_b_pc * initial_pose_offset_;

  mutex_.lock();
  Tom_ = Tbm * ~Tbo;
  mutex_.unlock();
  std::cout << "Initial pose guess processed. Tom: " << Tom_ << std::endl;
}

MICPSensorPtr MICPLocalizationNode::loadSensor(
  ParamTree<rclcpp::Parameter>::SharedPtr sensor_params)
{
  MICPSensorPtr sensor;
  std::string sensor_name = sensor_params->name;

  rclcpp::Node::SharedPtr nh_sensor = this->create_sub_node("sensors/" + sensor_name);
  
  ParamTree<rclcpp::Parameter>::SharedPtr sensor_param_tree
    = get_parameter_tree(nh_sensor, "~");
  
  const std::string data_source = sensor_param_tree->at("data_source")->data->as_string();
  const std::string model_type = sensor_param_tree->at("model_type")->data->as_string();

  auto corr_params = sensor_param_tree->at("correspondences");
  const std::string corr_backend = corr_params->at("backend")->data->as_string();
  const std::string corr_type = corr_params->at("type")->data->as_string();

  rm::UmeyamaReductionConstraints umeyama_params;
  umeyama_params.max_dist = corr_params->at("max_dist")->data->as_double(); // TODO: adaptive

  float adaptive_max_dist_min = umeyama_params.max_dist;
  if(corr_params->exists("adaptive_max_dist_min"))
  {
    adaptive_max_dist_min = corr_params->at("adaptive_max_dist_min")->data->as_double();
  }

  if(corr_backend == "embree")
  {
    #ifdef RMCL_EMBREE

    std::shared_ptr<MICPSensorCPU> sensor_cpu;

    if(model_type == "spherical")
    {
      sensor_cpu = std::make_shared<MICPSphericalSensorCPU>(nh_sensor);
      if(corr_type == "RC")
      {
        sensor_cpu->correspondences_ = std::make_shared<RCCEmbreeSpherical>(map_embree_);
      } else if(corr_type == "CP") {
        sensor_cpu->correspondences_ = std::make_shared<CPCEmbree>(map_embree_);
      } else {
        // ERROR
        std::cout << "Correspondence Type not implemented: " << corr_type << " for backend " << corr_backend << std::endl;
        return sensor;
      }
    }
    else if(model_type == "pinhole")
    {
      sensor_cpu = std::make_shared<MICPPinholeSensorCPU>(nh_sensor);
      if(corr_type == "RC")
      {
        sensor_cpu->correspondences_ = std::make_shared<RCCEmbreePinhole>(map_embree_);
      } else if(corr_type == "CP") {
        sensor_cpu->correspondences_ = std::make_shared<CPCEmbree>(map_embree_);
      } else {
        // ERROR
        std::cout << "Correspondence Type not implemented: " << corr_type << " for backend " << corr_backend << std::endl;
        return sensor;
      }
    } 
    else if(model_type == "o1dn")
    {
      sensor_cpu = std::make_shared<MICPO1DnSensorCPU>(nh_sensor);
      if(corr_type == "RC")
      {
        sensor_cpu->correspondences_ = std::make_shared<RCCEmbreeO1Dn>(map_embree_);
      } else if(corr_type == "CP") {
        sensor_cpu->correspondences_ = std::make_shared<CPCEmbree>(map_embree_);
      } else {
        // ERROR
        std::cout << "Correspondence Type not implemented: " << corr_type << " for backend " << corr_backend << std::endl;
        return sensor;
      }
    }
    else if(model_type == "ondn")
    {
      sensor_cpu = std::make_shared<MICPOnDnSensorCPU>(nh_sensor);
      if(corr_type == "RC")
      {
        sensor_cpu->correspondences_ = std::make_shared<RCCEmbreeOnDn>(map_embree_);
      } else if(corr_type == "CP") {
        sensor_cpu->correspondences_ = std::make_shared<CPCEmbree>(map_embree_);
      } else {
        // ERROR
        std::cout << "Correspondence Type not implemented: " << corr_type << " for backend " << corr_backend << std::endl;
        return sensor;
      }
    }
    else 
    {
      RCLCPP_ERROR_STREAM(get_logger(), "Unknown sensor model (embree) '" << model_type << "'.");
      return sensor;
    }
    
    if(!sensor_cpu)
    {
      RCLCPP_ERROR_STREAM(get_logger(), "Unknown error in sensor initialization (embree) '" << sensor_name << "'.");
      return sensor;
    }

    sensor_cpu->correspondences_->params = umeyama_params;
    sensor_cpu->correspondences_->adaptive_max_dist_min = adaptive_max_dist_min;
    sensor = sensor_cpu;
    
    #else
    throw std::runtime_error("backend 'embree' not compiled / not found");
    #endif // RMCL_EMBREE
  }
  else if(corr_backend == "optix")
  {
    #ifdef RMCL_OPTIX

    std::shared_ptr<MICPSensorCUDA> sensor_gpu;
    
    if(model_type == "spherical")
    {
      sensor_gpu = std::make_shared<MICPSphericalSensorCUDA>(nh_sensor);
      if(corr_type == "RC")
      {
        sensor_gpu->correspondences_ = std::make_shared<RCCOptixSpherical>(map_optix_);
      } 
      else
      {
        // ERROR
        std::cout << "Correspondence Type not implemented: " << corr_type << " for backend " << corr_backend << std::endl;
        return sensor;
      }
    }
    else if(model_type == "pinhole")
    {
      sensor_gpu = std::make_shared<MICPPinholeSensorCUDA>(nh_sensor);
      if(corr_type == "RC")
      {
        sensor_gpu->correspondences_ = std::make_shared<RCCOptixPinhole>(map_optix_);
      }
      else
      {
        // ERROR
        std::cout << "Correspondence Type not implemented: " << corr_type << " for backend " << corr_backend << std::endl;
        return sensor;
      }
    }
    else if(model_type == "o1dn")
    {
      sensor_gpu = std::make_shared<MICPO1DnSensorCUDA>(nh_sensor);
      if(corr_type == "RC")
      {
        sensor_gpu->correspondences_ = std::make_shared<RCCOptixO1Dn>(map_optix_);
      } 
      else
      {
        // ERROR
        std::cout << "Correspondence Type not implemented: " << corr_type << " for backend " << corr_backend << std::endl;
        return sensor;
      }
    }
    else if(model_type == "ondn")
    {
      sensor_gpu = std::make_shared<MICPOnDnSensorCUDA>(nh_sensor);
      if(corr_type == "RC")
      {
        sensor_gpu->correspondences_ = std::make_shared<RCCOptixOnDn>(map_optix_);
      } 
      else
      {
        // ERROR
        std::cout << "Correspondence Type not implemented: " << corr_type << " for backend " << corr_backend << std::endl;
        return sensor;
      }
    }
    else 
    {
      RCLCPP_ERROR_STREAM(get_logger(), "Unknown sensor model (optix) '" << model_type << "'.");
      return sensor;
    }

    if(!sensor_gpu)
    {
      RCLCPP_ERROR_STREAM(get_logger(), "Unknown error in sensor initialization (optix) '" << sensor_name << "'.");
      return sensor;
    }

    sensor_gpu->correspondences_->params = umeyama_params;
    sensor_gpu->correspondences_->adaptive_max_dist_min = adaptive_max_dist_min;
    sensor = sensor_gpu;

    #else
    throw std::runtime_error("backend 'optix' not compiled / not found");
    #endif // RMCL_OPTIX
  } 
  else 
  {
    RCLCPP_ERROR_STREAM(get_logger(), "Backend '" << corr_backend << "' not implemented.");
    throw std::runtime_error("Backend not known");
  }
  
  // at this point a sensor must be initialized
  if(!sensor)
  {
    RCLCPP_ERROR_STREAM(get_logger(), "Unknown error in sensor initialization '" << sensor_name << "'.");
    return sensor;
  }

  if(data_source == "topic")
  {
    const std::string topic_name = sensor_param_tree->at("topic_name")->data->as_string();
    sensor->connectToTopic(topic_name);
    sensor->on_data_received = std::bind(&MICPLocalizationNode::sensorDataReceived, this, std::placeholders::_1);
  }
  else if(data_source == "parameters") 
  {
    sensor->getDataFromParameters();
  } else {
    RCLCPP_ERROR_STREAM(get_logger(), "Data Source '" << data_source << "' not implemented.");
    throw std::runtime_error("Data source not known");
  }

  return sensor;
}

void MICPLocalizationNode::sensorDataReceived(
  const MICPSensorBase* sensor)
{
  data_stamp_latest_ = sensor->dataset_stamp_;
  Tbo_latest_ = sensor->Tbo;
  Tbo_stamp_latest_ = sensor->Tbo_stamp;
}

bool MICPLocalizationNode::fetchTF(
  const rclcpp::Time stamp)
{
  try {
    if(tf_buffer_->canTransform(
      odom_frame_, base_frame_, stamp, rclcpp::Duration::from_seconds(1.0)))
    {
      geometry_msgs::msg::TransformStamped T_base_odom 
        = tf_buffer_->lookupTransform(
            odom_frame_, base_frame_, stamp);
      Tbo_stamp_latest_ = T_base_odom.header.stamp;
      convert(T_base_odom.transform, Tbo_latest_);
    }
    else
    {
      std::cout << "micp_localization fetchTF ELSE" << std::endl;
      RCLCPP_WARN(this->get_logger(), "Transform not available yet.");
      return false;
    }
  } catch (tf2::TransformException& ex) {
    // std::cout << "Range sensor data is newer than odom! This not too bad. Try to get latest stamp" << std::endl;
    RCLCPP_WARN(get_logger(), "%s", ex.what());
    RCLCPP_WARN_STREAM(get_logger(), "Source (Base): " 
      << base_frame_ << ", Target (Odom): " << odom_frame_);
    return false;
  }
  return true;
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
  mutex_.lock();
  const rm::Transform Tom = Tom_;
  
  rmcl_msgs::msg::MICPSensorStats stats;
  stats.total_measurements = 0;
  stats.valid_measurements = 0;

  // #pragma omp parallel for
  for(auto sensor : sensors_vec_)
  {
    // dont change the state of this sensor
    sensor->mutex().lock();
    stats.total_measurements += sensor->total_dataset_measurements;
    stats.valid_measurements += sensor->valid_dataset_measurements;
  }

  stats.measurement_stamp = data_stamp_latest_;

  if(num_dynamic_sensors_ == 0)
  {
    for(auto sensor : sensors_vec_)
    {
      if(!sensor->fetchTF(now()))
      {
        std::cout << "Couldn't fetch tf from " << sensor->name << std::endl;
        return;
      }
    }
  } else {
    for(auto sensor : sensors_vec_)
    {
      if(sensor->static_dataset)
      {
        if(!sensor->fetchTF(data_stamp_latest_))
        {
          std::cout << "Couldn't fetch tf from " << sensor->name << std::endl;
          return;
        }
      }
    }
  }

  // collect infos
  for(auto sensor : sensors_vec_)
  {
    sensor->setTom(Tom);
    sensor->findCorrespondences();
    if(sensor->enable_visualizations)
    {
      sensor->drawCorrespondences();
    }
  }

  rm::Transform T_onew_oold = rm::Transform::Identity();

  rm::CrossStatistics Cmerged_o;
  rm::CrossStatistics Cmerged_weighted_o;
  
  for(size_t i=0; i<optimization_iterations_; i++)
  {
    // std::cout << "Correct! " << i << std::endl;
    Cmerged_o = rm::CrossStatistics::Identity();
    Cmerged_weighted_o = rm::CrossStatistics::Identity();

    bool outdated = false;

    for(const auto& sensor : sensors_vec_)
    { 
      // transform delta from odom frame to base frame, at time of respective sensor
      const rm::Transform T_bnew_bold = ~sensor->Tbo * T_onew_oold * sensor->Tbo;

      const rm::CrossStatistics Cs_b 
        = sensor->computeCrossStatistics(T_bnew_bold, convergence_progress_);

      const rm::CrossStatistics Cs_o = sensor->Tbo * Cs_b;

      rm::CrossStatistics Cs_weighted_o = Cs_o;
      Cs_weighted_o.n_meas *= sensor->merge_weight_multiplier;

      Cmerged_o += Cs_o; // optimal merge in odom frame
      Cmerged_weighted_o += Cs_weighted_o;
    }

    // checkStats(Cmerged_o);
    if(outdated)
    {
      break;
    }

    if(disable_correction_)
    {
      break;
    }
    
    // Cmerged_o -> T_onew_oold
    rm::Transform T_onew_oold_inner 
      = rm::umeyama_transform(Cmerged_o);

    if(!check(T_onew_oold_inner))
    {
      std::cout << "Malformed T_onew_oold_inner! " << T_onew_oold_inner.t << ", " << T_onew_oold_inner.R << std::endl;
    }

    // update T_onew_oold: 
    // transform from new odom frame to old odom frame
    // this is only virtual (a trick). we dont't want to change the odom frame in the end (instead the odom to map transform)
    T_onew_oold = T_onew_oold * T_onew_oold_inner;
  }

  for(auto sensor : sensors_vec_)
  {
    sensor->mutex().unlock();
  }

  // we want T_onew_map, we have Tom which is T_oold_map
  const rm::Transform T_onew_map = Tom * T_onew_oold;

  if(!disable_correction_ && Cmerged_o.n_meas > 0)
  {
    if(!check(T_onew_map))
    {
      std::cerr << "Malformed T_onew_map!" << std::endl;
    } 

    // store/update Tom, if allowed
    Tom_ = T_onew_map;
    Tom_.R.normalizeInplace();
  }

  mutex_.unlock();

  if(Cmerged_o.n_meas == 0 || !adaptive_max_dist_)
  {
    convergence_progress_ = 0.0;
  } 
  else 
  {
    const float trans_force = T_onew_map.t.l2norm();
    const float trans_progress = 1.0 / exp(10.0 * trans_force);

    const rm::Quaternion qunit = rm::Quaternion::Identity();
    float qscalar = T_onew_map.R.dot(qunit);
    float rot_progress = qscalar * qscalar;

    const double match_ratio = static_cast<double>(Cmerged_o.n_meas) / static_cast<double>(stats.valid_measurements);
    float adaption_rate = trans_progress * rot_progress * match_ratio;

    // Certainty
    // the certainty about this can be infered by the total number of measurements taken
    convergence_progress_ = adaption_rate;
  }

  { // publish stats
    stats.valid_matches = Cmerged_o.n_meas;
    stats.cov_trace = Cmerged_o.covariance.trace();
    stats.header.stamp = this->now();
    correction_stats_latest_ = stats;
    stats_publisher_->publish(stats);
  }
}

void MICPLocalizationNode::broadcastTransform()
{
  geometry_msgs::msg::TransformStamped T_odom_map;
  
  if(tf_time_source_ == 0)
  {
    T_odom_map.header.stamp = data_stamp_latest_;
  }
  else if(tf_time_source_ == 1)
  {
    T_odom_map.header.stamp = this->now();
  }
  else
  {
    // ERROR
    std::cerr << "ERROR UNKNOWN TF TIME SOURCE: " << tf_time_source_ << std::endl;
    return;
  }
  
  T_odom_map.header.frame_id = map_frame_;
  T_odom_map.child_frame_id = odom_frame_;

  // check Tom
  if(!check(Tom_))
  {
    std::cout << "[WARNING] Tom is malformed! : " << Tom_.t << ", " << Tom_.R << std::endl;
    // throw std::runtime_error("Tom malformed");
  }
  convert(Tom_, T_odom_map.transform);
  mutex_.unlock();
  

  tf_broadcaster_->sendTransform(T_odom_map);
}

void MICPLocalizationNode::publishPose()
{
  // mutex_.lock();

  geometry_msgs::msg::PoseWithCovarianceStamped pose;

  pose.header.stamp = data_stamp_latest_;
  pose.header.frame_id = map_frame_;

  // just some bad guess of the covariance
  // const double matched_ratio = static_cast<double>(correction_stats_latest_.valid_matches) / static_cast<double>(correction_stats_latest_.valid_measurements);
  // const double mean_error = correction_stats_latest_.cov_trace;
  // const double matched_ratio = 

  const double XX = (1.0 - convergence_progress_) + pose_noise_;

  pose.pose.covariance = {
    XX, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, XX, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, XX, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, XX/4.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, XX/4.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, XX/4.0
  };
  
  const rm::Transform Tbm = Tom_ * Tbo_latest_;
  rmcl::convert(Tbm, pose.pose.pose);

  // mutex_.unlock();

  Tbm_publisher_->publish(pose);
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
    if(!all_first_message_received)
    {
      std::cout << "Waiting for first messages to appear..." << std::endl;
      this->get_clock()->sleep_for(std::chrono::duration<double>(0.1));
    }
  }

  std::cout << "Every sensor recevied first message! Starting correction" << std::endl;

  rm::StopWatch sw;
  // double el = sw();
  
  double runtime_avg = 0.001;
  double new_factor = 0.1;

  const double desired_correction_time = 1.0 / correction_rate_max_;

  while(rclcpp::ok() && !stop_correction_thread_)
  {
    // RCLCPP_INFO_STREAM(this->get_logger(), "Latest data received vs now: " << data_stamp_latest_.seconds() << " vs " << this->get_clock()->now().seconds());
    
    // TODO: what if we have different computing units?
    // RTX have a smaller bottleneck vs Embree
    // We could do this in parallel

    sw();
    correctOnce();

    // if(broadcast_tf_)
    // {
    //   broadcastTransform();
    // }

    // if(publish_pose_)
    // {
    //   publishPose();
    // }

    const double el = sw();
    runtime_avg += (el - runtime_avg) * new_factor;
    const double wait_delay = desired_correction_time - runtime_avg;

    if(wait_delay > 0.0)
    {
      // wait
      this->get_clock()->sleep_for(std::chrono::duration<double>(wait_delay));
    }
  }
}

void MICPLocalizationNode::tfBroadcastLoop()
{
  double runtime_avg = 0.001;
  double new_factor = 0.1;

  rm::StopWatch sw;
  while(rclcpp::ok() && !stop_correction_thread_)
  {
    sw();
    broadcastTransform(); // this is the only important line
    const double el = sw();
    runtime_avg += (el - runtime_avg) * new_factor;
    const double desired_tf_time = 1.0 / tf_rate_;
    const double wait_delay = desired_tf_time - runtime_avg;
    if(wait_delay > 0.0)
    {
      // wait
      this->get_clock()->sleep_for(std::chrono::duration<double>(wait_delay));
    }
  }
}

} // namespace rmcl

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rmcl::MICPLocalizationNode)
