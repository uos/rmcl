#include "rmcl_ros/nodes/micp_localization.hpp"

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


#include <rmcl_ros/correction/sensors/MICPO1DnSensorCPU.hpp>

#ifdef RMCL_EMBREE
#include <rmcl_ros/correction/correspondences/RCCEmbree.hpp>
#include <rmcl_ros/correction/correspondences/CPCEmbree.hpp>
#endif // RMCL_EMBREE

#ifdef RMCL_CUDA
#include <rmcl_ros/correction/sensors/MICPO1DnSensorCUDA.hpp>
#include <rmcl_ros/correction/correspondences/RCCOptix.hpp>

#endif // RMCL_CUDA

#include <rmcl_msgs/msg/micp_stats.hpp>
#include <rmcl_msgs/msg/micp_sensor_stats.hpp>


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

  disable_correction_ = rmcl::get_parameter(this, "disable_correction", false);
  tf_time_source_ = rmcl::get_parameter(this, "tf_time_source", 0);
  optimization_iterations_ = rmcl::get_parameter(this, "optimization_iterations", 5);
  correction_rate_max_ = rmcl::get_parameter(this, "correction_rate_max", 1000.0);

  broadcast_tf_ = rmcl::get_parameter(this, "broadcast_tf", true);
  publish_pose_ = rmcl::get_parameter(this, "publish_pose", false);

  #ifdef RMCL_EMBREE
  map_embree_ = rm::import_embree_map(map_filename_);
  #endif // RMCL_EMBREE
  #ifdef RMCL_OPTIX
  map_optix_ = rm::import_optix_map(map_filename_);
  #endif // RMCL_OPTIX
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

  // incoming pose this needs to be synced with tf
  stats_publisher_ = this->create_publisher<rmcl_msgs::msg::MICPSensorStats>("micpl_stats", 10);

  pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/initialpose", 10, std::bind(&MICPLocalizationNode::poseCB, this, std::placeholders::_1));

  if(publish_pose_)
  {
    Tbm_publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("micpl_pose", 10);
  }

  std::cout << "Waiting for pose..." << std::endl;

  // timer. get latest tf
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

  // wait for correction loop to finish
  
  // normally the pose is set in map coords
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
  
  // TODO: remove this
  // update sensors
  // for(MICPSensorPtr& sensor : sensors_vec_)
  // {
  //   const rm::Transform Tom = Tbm * ~sensor->Tbo;
  //   sensor->setTom(Tom); // Tbm = Tom * Tbo
  // }
  
  this->Tom = Tbm * ~Tbo_latest_;
  
  std::cout << "Initial pose guess processed." << std::endl;
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
  
  // if(!sensor_param_tree->exists("type"))
  // {
  //   // ERROR!
  //   throw std::runtime_error("PARAM ERROR");
  // }
  
  // fetch parameters that decide which implementation is loaded
  // const std::string sensor_type = sensor_param_tree->at("type")->data->as_string();
  // std::cout << "- Type: " << sensor_type << std::endl;
  // const std::string model_source = sensor_param_tree->at("model_source")->data->as_string();  
  // std::cout << "- Model Source: " << model_source << std::endl;
  const std::string data_source = sensor_param_tree->at("data_source")->data->as_string();
  std::cout << "- Data Source: " << data_source << std::endl;

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

  std::cout << "- Correspondence Backend: " << corr_backend << std::endl;

  if(data_source == "topic")
  {
    const std::string topic_name = sensor_param_tree->at("topic")->at("name")->data->as_string();
    const std::string topic_type = sensor_param_tree->at("topic")->at("type")->data->as_string();

    if(topic_type == "rmcl_msgs/msg/O1DnStamped")
    {
      std::cout << "- Backend: " << corr_backend << std::endl;
      
      if(corr_backend == "embree")
      {
        #ifdef RMCL_EMBREE
        auto sensor_cpu = std::make_shared<MICPO1DnSensorCPU>(nh_sensor, topic_name);

        if(corr_type == "RC")
        {
          auto rcc_embree = std::make_shared<RCCEmbreeO1Dn>(map_embree_);
          rcc_embree->params = umeyama_params;
          rcc_embree->adaptive_max_dist_min = adaptive_max_dist_min;
          sensor_cpu->correspondences_ = rcc_embree;
        } 
        else if(corr_type == "CP") 
        {
          auto cpc_embree = std::make_shared<CPCEmbree>(map_embree_);
          cpc_embree->params = umeyama_params;
          cpc_embree->adaptive_max_dist_min = adaptive_max_dist_min;
          sensor_cpu->correspondences_ = cpc_embree;
        } 
        else 
        {
          // ERROR
          std::cout << "Correspondence Type not implemented: " << corr_type << " for backend " << corr_backend << std::endl;
          return sensor;
        }

        sensor_cpu->on_data_received = std::bind(&MICPLocalizationNode::sensorDataReceived, this, std::placeholders::_1);
      
        sensor = sensor_cpu;
        #else
        throw std::runtime_error("backend 'embree' not compiled / not found");
        #endif // RMCL_EMBREE
      }

      if(corr_backend == "optix")
      {
        #ifdef RMCL_OPTIX
        auto sensor_gpu = std::make_shared<MICPO1DnSensorCUDA>(nh_sensor, topic_name);

        if(corr_type == "RC")
        {
          auto rcc_optix = std::make_shared<RCCOptixO1Dn>(map_optix_);
          rcc_optix->params = umeyama_params;
          rcc_optix->adaptive_max_dist_min = adaptive_max_dist_min;
          sensor_gpu->correspondences_ = rcc_optix;
        } else {
          std::cout << "Correspondence Type not implemented: " << corr_type << " for backend " << corr_backend << std::endl;
          return sensor;
        }
        
        sensor_gpu->on_data_received = std::bind(&MICPLocalizationNode::sensorDataReceived, this, std::placeholders::_1);
      
        sensor = sensor_gpu;
        #else
        throw std::runtime_error("backend 'optix' not compiled / not found");
        #endif // RMCL_OPTIX
      }
      
    }
  } else {
    RCLCPP_ERROR_STREAM(get_logger(), "Topic source '" << data_source << "' not implemented.");
    throw std::runtime_error("Topic source not known");
  }

  if(sensor)
  {
    // set frames
    sensor->base_frame = base_frame_;
    sensor->odom_frame = odom_frame_;
    sensor->map_frame = map_frame_;
  }

  return sensor;
}

void MICPLocalizationNode::sensorDataReceived(
  const MICPSensorBase* sensor)
{
  // std::cout << sensor->name << " received data!" << std::endl;
  data_stamp_latest_ = sensor->dataset_stamp_;
  Tbo_latest_ = sensor->Tbo;
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

  // collect infos

  // #pragma omp parallel for
  for(auto sensor : sensors_vec_)
  {
    // only set current transform from odom to map
    // Tbo and Tsb are fetched synchron to the arriving sensor data
    sensor->setTom(Tom);
    sensor->findCorrespondences();
  }

  rm::Transform T_onew_oold = rm::Transform::Identity();

  rm::CrossStatistics Cmerged_o;
  rm::CrossStatistics Cmerged_weighted_o;
  
  for(size_t i=0; i<optimization_iterations_; i++)
  {
    // std::cout << "Correct! " << correction_counter << ", " << i << std::endl;
    Cmerged_o = rm::CrossStatistics::Identity();
    Cmerged_weighted_o = rm::CrossStatistics::Identity();

    bool outdated = false;

    // #pragma omp parallel for
    for(const auto sensor : sensors_vec_)
    {
      if(sensor->correspondencesOutdated())
      {
        RCLCPP_DEBUG_STREAM(get_logger(), "Correspondences outdated!");
        outdated = true;
        continue;
      }
      
      // transform delta from odom frame to base frame, at time of respective sensor
      const rm::Transform T_bnew_bold = ~sensor->Tbo * T_onew_oold * sensor->Tbo;

      const rm::CrossStatistics Cs_b 
        = sensor->computeCrossStatistics(T_bnew_bold, convergence_progress_);

      const float Cs_trace = Cs_b.covariance.trace();
      // std::cout << "Cs_trace: " << Cs_trace << std::endl;
      // std::cout << "Cmerged_o trace: " << Cmerged_o << std::endl;
      
      // std::cout << sensor->name << " : " 
      //   << Cs_b.n_meas << " valid matches, " 
      //   << Cs_trace / static_cast<double>(Cs_b.n_meas) << " normed trace." << std::endl;

      const rm::CrossStatistics Cs_o = sensor->Tbo * Cs_b;

      rm::CrossStatistics Cs_weighted_o = Cs_o;
      Cs_weighted_o.n_meas *= sensor->merge_weight_multiplier;

      #pragma omp critical
      {
        Cmerged_o += Cs_o; // optimal merge in odom frame
        Cmerged_weighted_o += Cs_weighted_o;
      }
    }

    if(outdated)
    {
      break;
    }

    if(disable_correction_)
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

  // #pragma omp parallel for
  for(auto sensor : sensors_vec_)
  {
    sensor->mutex().unlock();
  }

  // we want T_onew_map, we have Tom which is T_oold_map
  const rm::Transform T_onew_map = Tom * T_onew_oold;

  if(!disable_correction_)
  {
    // store/update Tom, if allowed
    Tom = T_onew_map;
  }

  //  std::cout << "Cmerged_o trace: " << Cmerged_o.covariance << std::endl;

  if(Cmerged_o.n_meas == 0)
  {
    convergence_progress_ = 0.0;
  } else {
    const double max_dist = 2.0; // TODO: dynamic?

    // Ratio of Matched Data:
    // if map is perfect 
    // - this will be 1.0 if localized correctly
    // - this will be 0.0 if not localized at all
    // - this will be 1.0 if localized wrongly, but model is ambiguous (same looking rooms)
    const double matched_ratio = static_cast<double>(Cmerged_o.n_meas) / static_cast<double>(stats.valid_measurements);

    // Progress from Mean Error
    // if map is perfect
    // - this will be 1.0 if localized correctly
    // - this will be 0.0 if localized poorly
    // - this will
    const double mean_error = Cmerged_o.covariance.trace() / static_cast<double>(Cmerged_o.n_meas);
    const double progress_from_error = 1.0 - (std::clamp(mean_error, 0.0, max_dist) / max_dist);

    const double total_progress = pow(matched_ratio * progress_from_error, 1.0/2.0);

    // Certainty
    // the certainty about this can be infered by the total number of measurements taken
    convergence_progress_ = total_progress;
  }

  // std::cout << "Total Progress: " << convergence_progress_ << std::endl;
  
  { // publish stats
    stats.valid_matches = Cmerged_o.n_meas;
    stats.cov_trace = Cmerged_o.covariance.trace() / static_cast<double>(Cmerged_o.n_meas);
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
    std::cout << "ERROR UNKNOWN TF TIME SOURCE: " << tf_time_source_ << std::endl;
    return;
  }
  
  T_odom_map.header.frame_id = map_frame_;
  T_odom_map.child_frame_id = odom_frame_;
  convert(Tom, T_odom_map.transform);
  tf_broadcaster_->sendTransform(T_odom_map);
}

void MICPLocalizationNode::publishPose()
{
  geometry_msgs::msg::PoseWithCovarianceStamped pose;

  pose.header.stamp = data_stamp_latest_;
  pose.header.frame_id = map_frame_;

  // just some bad guess of the covariance
  const double matched_ratio = static_cast<double>(correction_stats_latest_.valid_matches) / static_cast<double>(correction_stats_latest_.valid_measurements);
  const double mean_error = correction_stats_latest_.cov_trace;
  const double XX = mean_error / matched_ratio;

  pose.pose.covariance = {
    XX, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, XX, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, XX, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, XX/4.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, XX/4.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, XX/4.0
  };
  
  rm::Transform Tbm = Tom * Tbo_latest_;
  rmcl::convert(Tbm, pose.pose.pose);
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

    if(broadcast_tf_)
    {
      broadcastTransform();
    }

    if(publish_pose_)
    {
      publishPose();
    }

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

} // namespace rmcl

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rmcl::MICPLocalizationNode)
