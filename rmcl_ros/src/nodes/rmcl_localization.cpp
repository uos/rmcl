#include "rmcl_ros/nodes/rmcl_localization.hpp"

#include <rmcl_ros/rmcl/TFMotionUpdaterCPU.hpp>
#include <rmcl_ros/rmcl/PCDSensorUpdaterEmbree.hpp>
#include <rmcl_ros/rmcl/KLDResamplerCPU.hpp>
#include <rmcl_ros/rmcl/ResidualResamplerCPU.hpp>
#include <rmcl_ros/rmcl/GladiatorResamplerCPU.hpp>

// GPU
#include <rmcl_ros/rmcl/TFMotionUpdaterGPU.hpp>
#include <rmcl_ros/rmcl/PCDSensorUpdaterOptix.hpp>
#include <rmcl_ros/rmcl/GladiatorResamplerGPU.hpp>

#include <rmagine/math/memory_math.h>


namespace rmcl
{

RmclNode::RmclNode(const rclcpp::NodeOptions& options)
: rclcpp::Node("rmcl_node", rclcpp::NodeOptions(options).allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true) )
{
  map_container_ = std::make_shared<rm::MapMap>();

  tf_buffer_ =
    std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ =
    std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ =
      std::make_shared<tf2_ros::TransformBroadcaster>(*this);
  
  motion_update_node_ = this->create_sub_node("motion_update");
  sensor_update_node_ = this->create_sub_node("sensor_update");
  resampling_node_ = this->create_sub_node("resampling");

  // general params
  updateGeneralParams();

  // initialization params
  updateInitializationParams();

  // general motion update params
  updateMotionUpdateParams();

  // sensor update params
  updateSensorUpdateParams();

  // sampling
  updateResamplingParams();

  // visualization
  updateVisualizationParams();


  sub_pose_wc_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    config_pose_initialization_.topic, 1, 
    [=](const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr& msg)
    {
      initSamples(*msg);
    });

  srv_global_initialization_ = this->create_service<std_srvs::srv::Empty>(
    "rmcl/global_localization",
    [=](const std::shared_ptr<rmw_request_id_t> /*header*/,
           const std::shared_ptr<std_srvs::srv::Empty::Request> /*req*/,
           std::shared_ptr<std_srvs::srv::Empty::Response> /*res*/) -> void
      {
        initSamplesUniform();
      });

  srv_pose_initialization_ = this->create_service<rmcl_msgs::srv::SetInitialPose>(
    "rmcl/initial_pose_guess",
    [=](const std::shared_ptr<rmw_request_id_t> /*header*/,
           const std::shared_ptr<rmcl_msgs::srv::SetInitialPose::Request> req,
           std::shared_ptr<rmcl_msgs::srv::SetInitialPose::Response> /*res*/) -> void
      {
        initSamples(req->pose);
      });

  pub_pose_wc_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("rmcl/pose", 1);

  // motion update
  motion_update_timer_ = this->create_wall_timer(
      std::chrono::duration<float>(1.0 / config_motion_update_.rate), [this](){ 
        motionUpdate(); 
      });

  // sensor update
  sub_pcl_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    config_sensor_update_.topic, 10, 
    [=](const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg)
    {
      sensorUpdate(msg);
    });

  if(config_resampling_.rate > 0.0001)
  {
    resampling_timer_ = this->create_wall_timer(
      std::chrono::duration<float>(1.0 / config_resampling_.rate), [this](){ resampling(); });
  }
  
  if(config_visualization_.enabled)
  {
    viz_timer_ = this->create_wall_timer(
      std::chrono::duration<float>(1.0 / config_visualization_.rate), [this](){ visualize(); });

    pub_pcl_viz_ = this->create_publisher<sensor_msgs::msg::PointCloud>("rmcl/particles", 10);
  }
  
  initMemory();
  initSamplesUniform();
}

void RmclNode::initMemory()
{
  // TODO
  // std::cout << "MAX THREADS: " << omp_get_max_threads() << std::endl;

  // CPU
  particle_cloud_0_.poses.resize(config_general_.max_particles);
  particle_cloud_0_.attrs.resize(config_general_.max_particles);

  particle_cloud_1_.poses.resize(config_general_.max_particles);
  particle_cloud_1_.attrs.resize(config_general_.max_particles);

  particle_cloud_       = &particle_cloud_0_;
  particle_cloud_next_  = &particle_cloud_1_;


  // GPU
  particle_cloud_gpu_0_.poses.resize(config_general_.max_particles);
  particle_cloud_gpu_0_.attrs.resize(config_general_.max_particles);

  particle_cloud_gpu_1_.poses.resize(config_general_.max_particles);
  particle_cloud_gpu_1_.attrs.resize(config_general_.max_particles);

  particle_cloud_gpu_       = &particle_cloud_gpu_0_;
  particle_cloud_gpu_next_  = &particle_cloud_gpu_1_;

}

float abssum(rm::Matrix6x6 C)
{
  float res = 0.0;
  for(size_t i=0; i<6; i++)
  {
    for(size_t j=0; j<6; j++)
    {
      res += abs(C(i, j));
    }
  }
  return res;
}

inline void convert(const std::array<double, 36>& Cin, rm::Matrix6x6& Cout)
{
  for(size_t i=0; i<6; i++)
  {
    for(size_t j=0; j<6; j++)
    {
      Cout(i, j) = Cin[i * 6 + j];
    }
  }
}

void RmclNode::initSamples(
  const geometry_msgs::msg::PoseWithCovarianceStamped& pose_wc)
{
  updateGeneralParams();
  updateInitializationParams();

  std::cout << "initSamples from pose!" << std::endl;


  size_t seed = this->now().nanoseconds();

  std::mt19937 rng(seed);
  std::normal_distribution dist{0.0, 1.0}; // std normal dist


  rm::Matrix6x6 C;
  convert(pose_wc.pose.covariance, C);

  // TODO: to transform the vector x of independent random variables sampled 
  // from the standard normal distribution using this covariance we need to 
  // determine A*x + mu with A*A' = C . This can be done using Cholesky decomposition with L=A

  rm::Matrix6x6 L;
  rm::chol(C, L);

  std::cout << "C:" << std::endl;
  std::cout << C << std::endl;

  std::cout << "Cholesky:" << std::endl;
  std::cout << L << std::endl;
  std::cout << "Cholesky Err: " << abssum(L*L.T() - C) / 36.0 << std::endl;

  rm::Matrix_<float, 6, 1> mu;
  mu(0,0) = pose_wc.pose.pose.position.x;
  mu(1,0) = pose_wc.pose.pose.position.y;
  mu(2,0) = pose_wc.pose.pose.position.z;

  // Transform sample from local coords to pose frame (pose_wc.header.stamp)
  rm::Transform Tlp;
  Tlp.R = {
    (float)pose_wc.pose.pose.orientation.x,
    (float)pose_wc.pose.pose.orientation.y,
    (float)pose_wc.pose.pose.orientation.z,
    (float)pose_wc.pose.pose.orientation.w
  };
  Tlp.t = {
    (float)pose_wc.pose.pose.position.x,
    (float)pose_wc.pose.pose.position.y,
    (float)pose_wc.pose.pose.position.z
  };


  // Transform samples from pose frame to map frame
  rm::Transform Tpm;
  // TODO: use tf to get the transform
  // Currently, it's only possible to set the pose in the map frame
  // which is OK until it will be possible to visualize the map in another frame
  Tpm = rm::Transform::Identity(); 

  // Transform from local to map = local -> pose -> map
  const rm::Transform Tlm = Tpm * Tlp;

  std::unique_lock lock(data_mtx_);

  // set new working cloud
  n_particles_ = std::min(config_general_.max_particles, config_pose_initialization_.n_particles);
  for(size_t i=0; i<n_particles_; i++)
  {
    rm::Matrix_<float, 6, 1> x;
    { // set random values from standard normal distribution
      x(0,0) = dist(rng);
      x(1,0) = dist(rng);
      x(2,0) = dist(rng);
      x(3,0) = dist(rng);
      x(4,0) = dist(rng);
      x(5,0) = dist(rng);
    }
    
    // deform dist using covariance
    x = L * x;
    const rm::Transform Pl{
      rm::EulerAngles{x(3,0), x(4,0), x(5,0)}, // R
      rm::Vector3f{x(0,0), x(1,0), x(2,0)}, // t
      0 // time stamp (mostly unused)
    };

    particle_cloud_->poses[i] = Tlm * Pl;
    ParticleAttributes p_attr;
    p_attr.likelihood = rm::Gaussian1D::Identity();
    p_attr.likelihood.mean = 1.0;
    particle_cloud_->attrs[i] = p_attr;
  }

  // forces copying data to GPU if required
  data_location_ = "cpu";

  updateGeneralParams();// as motion updater and sensor_updater possibly contain 
  // statistics over history of data, the simplest thing is to
  // force them to reinitialize.
  // --> this could also be convinient as it forces the program to change plugins
  motion_updater_.reset();
  motion_updater_gpu_.reset();
  motion_update_done_ = false;

  sensor_updater_.reset();
  sensor_update_done_ = false;

  resampler_.reset();

  std::cout << "New samples initialized!" << std::endl;
}

void RmclNode::initSamplesUniform()
{
  // 1M particles
  updateGeneralParams();

  updateInitializationParams();
  
  size_t seed = this->now().nanoseconds();

  std::mt19937 rng(seed);
  std::uniform_real_distribution<float> dist_x(config_global_initialization_.bb_min[0], config_global_initialization_.bb_max[0]);
  std::uniform_real_distribution<float> dist_y(config_global_initialization_.bb_min[1], config_global_initialization_.bb_max[1]);
  std::uniform_real_distribution<float> dist_z(config_global_initialization_.bb_min[2], config_global_initialization_.bb_max[2]);
  std::uniform_real_distribution<float> dist_roll(config_global_initialization_.bb_min[3], config_global_initialization_.bb_max[3]);
  std::uniform_real_distribution<float> dist_pitch(config_global_initialization_.bb_min[4], config_global_initialization_.bb_max[4]);
  std::uniform_real_distribution<float> dist_yaw(config_global_initialization_.bb_min[5], config_global_initialization_.bb_max[5]);

  // rm::Vector6f bla;

  // noise, per change, per second, per action?

  // rm::Matrix6x6 motion_noise = {{             // state:
  //             0.1, 1.0, 0.0, 0.0, 0.0, 0.0, // tx: driving forwards gives more uncertainty on the y axis
  //             1.0, 0.1, 0.0, 0.0, 0.0, 0.0, // ty: driving side-wards gives more uncertainty on the x axis
  //             1.0, 1.0, 1.0, 1.0, 1.0, 1.0, // tz
  //             0.0, 0.0, 0.0, 1.0, 0.0, 0.0, // roll
  //             0.0, 0.0, 0.0, 0.0, 1.0, 0.0, // pitch
  //             0.01, 0.01, 0.0, 0.0, 0.0, 1.0 // yaw
  // }};
  // motion:     x   y    z   roll pitch yaw

  // state noise delta (0) = 0.1 * motion(0) + 1.0 * motion(1)

  std::unique_lock lock(data_mtx_);

  // set new working cloud
  n_particles_ = std::min(config_general_.max_particles, config_global_initialization_.n_particles);

  for(size_t i=0; i<n_particles_; i++)
  {
    rm::Transform Tbm;
    Tbm.t = {dist_x(rng), dist_y(rng), dist_z(rng)};
    Tbm.R = rm::EulerAngles{dist_roll(rng), dist_pitch(rng), dist_yaw(rng)};
  
    particle_cloud_->poses[i] = Tbm;
    ParticleAttributes p_attr;
    p_attr.likelihood = rm::Gaussian1D::Identity();
    // keeping this zero can result in strange behavior when no sensor update can be done until the next resampling step
    p_attr.likelihood.mean = 1.0; 
    // p_attr.state_cov = cov_init;
    particle_cloud_->attrs[i] = p_attr;
  }
  // forces copying data to GPU if required
  data_location_ = "cpu";

  motion_updater_.reset();
  motion_updater_gpu_.reset();
  motion_update_done_ = false;

  sensor_updater_.reset();
  sensor_update_done_ = false;

  resampler_.reset();
  resampler_gpu_.reset();
  std::cout << "New samples initialized!" << std::endl;
}

void RmclNode::updateGeneralParams()
{
  config_general_.base_frame = rmcl::get_parameter(this, "~base_frame", "base_link");
  config_general_.odom_frame = rmcl::get_parameter(this, "~odom_frame", "odom");
  config_general_.map_frame  = rmcl::get_parameter(this, "~map_frame", "map");
  config_general_.max_particles = rmcl::get_parameter(this, "~max_particles", 1000000);
}

void RmclNode::updateInitializationParams()
{
  // pose initialization
  config_pose_initialization_.topic = rmcl::get_parameter(this, "initialization.pose.topic", "/initialpose");
  config_pose_initialization_.n_particles = rmcl::get_parameter(this, "initialization.pose.particles", 50000);
  
  // global initialization
  config_global_initialization_.n_particles = rmcl::get_parameter(this, "initialization.global.particles", 50000);
  config_global_initialization_.bb_min = rmcl::get_parameter(this, "initialization.global.box.min", config_global_initialization_.bb_min);
  config_global_initialization_.bb_max = rmcl::get_parameter(this, "initialization.global.box.max", config_global_initialization_.bb_max);
}

void RmclNode::updateMotionUpdateParams()
{
  config_motion_update_.type    = rmcl::get_parameter(motion_update_node_, "~type", "");
  config_motion_update_.compute = rmcl::get_parameter(motion_update_node_, "~compute", "");
  config_motion_update_.rate    = rmcl::get_parameter(motion_update_node_, "~rate", 50.0);
}

void RmclNode::updateSensorUpdateParams()
{
  config_sensor_update_.type = rmcl::get_parameter(sensor_update_node_, "~type", "");
  config_sensor_update_.compute = rmcl::get_parameter(sensor_update_node_, "~compute", "");
  config_sensor_update_.correspondence_type = rmcl::get_parameter(sensor_update_node_, "~correspondence_type", 0);
  config_sensor_update_.dist_sigma = rmcl::get_parameter(sensor_update_node_, "~dist_sigma", 2.0);
  config_sensor_update_.samples = rmcl::get_parameter(sensor_update_node_, "~samples", 100);
  config_sensor_update_.topic = rmcl::get_parameter(sensor_update_node_, "~topic", "points");
}

void RmclNode::updateResamplingParams()
{
  config_resampling_.type = rmcl::get_parameter(resampling_node_, "~type", "");
  config_resampling_.rate = rmcl::get_parameter(resampling_node_, "~rate", 20.0);
  config_resampling_.compute = rmcl::get_parameter(resampling_node_, "~compute", "");
  config_resampling_.max_induction_particles = rmcl::get_parameter(resampling_node_, "~max_induction_particles", 50000);
}

void RmclNode::updateVisualizationParams()
{
  config_visualization_.enabled = rmcl::get_parameter(this, "visualization.enabled", true);
  config_visualization_.rate = rmcl::get_parameter(this, "visualization.rate", 20.0);
  config_visualization_.max_particles = rmcl::get_parameter(this, "visualization.max_particles", 50000);
}

void RmclNode::prepareMemory(const std::string& target_device)
{
  if(data_location_ == target_device)
  {
    return;
  }

  if(target_device == "cpu")
  {
    if(data_location_ == "gpu")
    {
      // transfer data from GPU to CPU
      particle_cloud_->poses = particle_cloud_gpu_->poses;
      particle_cloud_->attrs = particle_cloud_gpu_->attrs;
      data_location_ = "cpu";
    } else {
      std::cout << "prepareMemory - ERROR 1" << std::endl;
    }
  } 
  else if(target_device == "gpu") 
  {
    if(data_location_ == "cpu") 
    {
      // transfer data from CPU to GPU
      particle_cloud_gpu_->poses = particle_cloud_->poses;
      particle_cloud_gpu_->attrs = particle_cloud_->attrs;
      data_location_ = "gpu";
    } else {
      std::cout << "prepareMemory - ERROR 2" << std::endl;
    }
  } else {
    std::cout << "prepareMemory - ERROR X" << std::endl;
  }

}

void RmclNode::motionUpdate()
{
  std::unique_lock lock(data_mtx_);

  std::cout << "-------------------" << std::endl;
  std::cout << "{ // Motion Update" << std::endl;

  if(!motion_updater_)
  {
    // TODO: check if we can apply the pluginlib here
    if(config_motion_update_.type == "tf")
    {
      motion_updater_ = std::make_shared<TFMotionUpdaterCPU>(
        map_container_,
        motion_update_node_,
        tf_buffer_
      );
      motion_updater_gpu_ = std::make_shared<TFMotionUpdaterGPU>(
        map_container_,
        motion_update_node_,
        tf_buffer_
      );
    } else {
      // ERROR
      throw std::runtime_error("ERROR motion update type unknown");
    }

    // declare parameters etc
    motion_updater_->init();
    motion_updater_gpu_->init();
  }

  prepareMemory(config_motion_update_.compute);

  // update
  if(config_motion_update_.compute == "cpu")
  {
    motion_updater_->update(
      particle_cloud_->poses(0, n_particles_),
      particle_cloud_->attrs(0, n_particles_)
    );
  } else if(config_motion_update_.compute == "gpu") {
    motion_updater_gpu_->update(
      particle_cloud_gpu_->poses(0, n_particles_),
      particle_cloud_gpu_->attrs(0, n_particles_)
    );
  } else {
    std::cout << "   ERROR: param value not known for 'compute': " << config_motion_update_.compute << std::endl;
  }

  motion_update_done_ = true;
  std::cout << "} // Motion Update" << std::endl;
}

void RmclNode::sensorUpdate(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg)
{
  std::unique_lock lock(data_mtx_);

  std::cout << "-------------------" << std::endl;
  std::cout << "{ // Sensor Update" << std::endl;

  if(!sensor_updater_)
  {
    // TODO: check if we can apply the pluginlib here
    if(config_sensor_update_.type == "pcd")
    {
      // initialize all pcd types here
      sensor_updater_ = std::make_shared<PCDSensorUpdaterEmbree>(
        map_container_,
        sensor_update_node_,
        tf_buffer_
      );
      sensor_updater_gpu_ = std::make_shared<PCDSensorUpdaterOptix>(
        map_container_,
        sensor_update_node_,
        tf_buffer_
      );
    } else {
      // ERROR
      throw std::runtime_error("ERROR sensor update type unknown");
    }

    // declare parameters etc
    sensor_updater_->init();
    sensor_updater_gpu_->init();
  }

  { // set input
    auto input = std::dynamic_pointer_cast<Input<sensor_msgs::msg::PointCloud2::ConstSharedPtr> >(sensor_updater_);
    if(!input)
    {
      RCLCPP_INFO(this->get_logger(), "Sensor Updater has wrong input!");
      return;
    }
    input->setInput(msg);
  }

  { // set input GPU
    auto input = std::dynamic_pointer_cast<Input<sensor_msgs::msg::PointCloud2::ConstSharedPtr> >(sensor_updater_gpu_);
    if(!input)
    {
      RCLCPP_INFO(this->get_logger(), "Sensor Updater has wrong input!");
      return;
    }
    input->setInput(msg);
  }

  prepareMemory(config_sensor_update_.compute);

  if(config_sensor_update_.compute == "cpu")
  {
    // RCLCPP_INFO(this->get_logger(), "Update!");
    // update
    sensor_updater_->update(
      particle_cloud_->poses(0, n_particles_),
      particle_cloud_->attrs(0, n_particles_)
    );
  } else if(config_sensor_update_.compute == "gpu") {
    sensor_updater_gpu_->update(
      particle_cloud_gpu_->poses(0, n_particles_),
      particle_cloud_gpu_->attrs(0, n_particles_)
    );
  } else {
    std::cout << "   ERROR: param value not known for 'compute': " << config_sensor_update_.compute << std::endl;
  }  

  sensor_update_done_ = true;

  std::cout << "} // Sensor Update " << std::endl;
}

void RmclNode::resampling()
{

  induceState(); // prototype. uncomment this for just computing the particle set


  std::unique_lock lock(data_mtx_);

  

  std::cout << "-------------------" << std::endl;
  std::cout << "{ // Resampling" << std::endl;
  if(!resampler_)
  {
    // TODO: check if we can apply the pluginlib here
    if(config_resampling_.type == "residual")
    {
      resampler_ = std::make_shared<ResidualResamplerCPU>(
        resampling_node_
      );

      // resampler_gpu_ = std::make_shared<ResidualResamplerGPU>(
      //   resampling_node_
      // );
    } 
    else if(config_resampling_.type == "gladiator") 
    {
      resampler_ = std::make_shared<GladiatorResamplerCPU>(
        resampling_node_
      );
      resampler_gpu_ = std::make_shared<GladiatorResamplerGPU>(
        resampling_node_
      );
    } 
    else 
    {
      // ERROR
      throw std::runtime_error("ERROR sensor update type unknown");
    }

    // declare parameters etc
    resampler_->init();
  }

  if(!motion_update_done_ || !sensor_update_done_)
  {
    std::cout << "   Resampling: Waiting for at least one motion update or sensor_update to be done." << std::endl;
    return;
  }

  ParticleUpdateDynamicResults res;

  ParticleUpdateDynamicConfig config;

  prepareMemory(config_resampling_.compute);

  if(config_resampling_.compute == "cpu")
  { 
    res = resampler_->update(
      particle_cloud_->poses(0, n_particles_),
      particle_cloud_->attrs(0, n_particles_),
      particle_cloud_next_->poses,
      particle_cloud_next_->attrs,
      config
    );

    std::swap(particle_cloud_, particle_cloud_next_);

  } else if(config_resampling_.compute == "gpu") {
    
    std::cout << "   - GPU" << std::endl;
    res = resampler_gpu_->update(
      particle_cloud_gpu_->poses(0, n_particles_),
      particle_cloud_gpu_->attrs(0, n_particles_),
      particle_cloud_gpu_next_->poses,
      particle_cloud_gpu_next_->attrs,
      config
    );

    std::swap(particle_cloud_gpu_, particle_cloud_gpu_next_);

    // prepareMemory("cpu");
    // data_location_ = "gpu";

    // std::cout << "Particles Debug:" << std::endl;
    // for(size_t i=0; i<n_particles_; i++)
    // {
    //   const rm::Transform pose = particle_cloud_->poses[0];
    //   std::cout << pose.t.x << ", " << pose.t.y << ", " << pose.t.z << std::endl;
    // }

  } else {
    std::cout << "   ERROR: param value not known for 'compute': " << config_sensor_update_.compute << std::endl;
  }

  std::cout << "   DONE." << std::endl;

  std::cout << "   Particles: " << n_particles_ << " -> " << res.n_particles << std::endl;

  if(res.n_particles < 10)
  {
    std::cout << "   ERROR: NUMBER OF PARTICLES TO LOW AFTER RESAMPLING! : " << res.n_particles << " -> Skipping Resampling step." << std::endl;
    return;
  }

  n_particles_ = res.n_particles;

  std::cout << "   Swapped." << std::endl;

  std::cout << "} // Resampling" << std::endl;

  // induceState();

  // visualize();
}

void RmclNode::induceState()
{
  std::shared_lock lock(data_mtx_);

  std::cout << "-------------------" << std::endl;
  std::cout << "{ // State Induction" << std::endl;


  rm::StopWatch sw;
  double el;

  sw();

  // make sync cpu memory
  prepareMemory("cpu");
  // copy cpu memory to this function
  // ParticleCloud<rm::RAM> particle_cloud = *particle_cloud_;

  const size_t n_induction_particles = std::min(n_particles_, config_resampling_.max_induction_particles);

  // work slice
  rm::MemoryView<rm::Transform, rm::RAM> particle_poses = particle_cloud_->poses(0, n_induction_particles);
  rm::MemoryView<ParticleAttributes, rm::RAM> particle_attrs = particle_cloud_->attrs(0, n_induction_particles);

  el = sw();

  std::cout << "   - sync memory: " << el << "s" << std::endl;

  // weight stats
  sw();
  double L_sum = 0.0;
  double L_sum_sq = 0.0;
  double L_max = 0.0;
  double L_min = std::numeric_limits<double>::max();
  double L_n = static_cast<double>(particle_attrs.size());

  for(size_t i=0; i<particle_attrs.size(); i++)
  {
    const double v = particle_attrs[i].likelihood.mean;
    L_sum += v;
    L_sum_sq += v*v;
    L_max = std::max(L_max, v);
    L_min = std::min(L_min, v);
  }

  double L_mean = L_sum / L_n;
  double L_var  = L_sum_sq / L_n - L_mean * L_mean;

  el = sw();
  std::cout << "   - weight stats: " << el << "s" << std::endl;

  sw();
  // geom stats
  // rm::Transform Tbm = rm::karcher_mean(particle_poses, [&](size_t i){
  //   return particle_attrs[i].likelihood.mean / L_sum;
  // });

  rm::Transform Tbm = rm::mock_mean(particle_poses, [&](size_t i){
    return particle_attrs[i].likelihood.mean / L_sum;
  });

  el = sw();

  std::cout << "   - weight-aware geom stats: " << el << "s" << std::endl;
  std::cout << "   - Induced Pose: " << Tbm << std::endl;

  geometry_msgs::msg::PoseWithCovarianceStamped Pbm;

  Pbm.header.frame_id = "map";
  Pbm.header.stamp = this->now();

  Pbm.pose.pose.position.x = Tbm.t.x;
  Pbm.pose.pose.position.y = Tbm.t.y;
  Pbm.pose.pose.position.z = Tbm.t.z;
  Pbm.pose.pose.orientation.x = Tbm.R.x;
  Pbm.pose.pose.orientation.y = Tbm.R.y;
  Pbm.pose.pose.orientation.z = Tbm.R.z;
  Pbm.pose.pose.orientation.w = Tbm.R.w;

  pub_pose_wc_->publish(Pbm);

  { // broadcast transform
    
    // 1. get current transform between base and odom
    try {
      geometry_msgs::msg::TransformStamped Tbo_ros = tf_buffer_->lookupTransform(
        config_general_.odom_frame, // to
        config_general_.base_frame, // from
        this->now(), // TODO: is this right?
        tf2::durationFromSec(0.3));

      // T_bnew_o_stamp = T.header.stamp;
      
      rm::Transform Tbo;
      Tbo.t.x = Tbo_ros.transform.translation.x;
      Tbo.t.y = Tbo_ros.transform.translation.y;
      Tbo.t.z = Tbo_ros.transform.translation.z;
      Tbo.R.x = Tbo_ros.transform.rotation.x;
      Tbo.R.y = Tbo_ros.transform.rotation.y;
      Tbo.R.z = Tbo_ros.transform.rotation.z;
      Tbo.R.w = Tbo_ros.transform.rotation.w;

      const rm::Transform Tom = Tbm * ~Tbo;
      
      geometry_msgs::msg::TransformStamped Tom_ros;
      Tom_ros.transform.translation.x = Tom.t.x;
      Tom_ros.transform.translation.y = Tom.t.y;
      Tom_ros.transform.translation.z = Tom.t.z;
      Tom_ros.transform.rotation.x = Tom.R.x;
      Tom_ros.transform.rotation.y = Tom.R.y;
      Tom_ros.transform.rotation.z = Tom.R.z;
      Tom_ros.transform.rotation.w = Tom.R.w;

      Tom_ros.header.frame_id = config_general_.map_frame;
      Tom_ros.child_frame_id = config_general_.odom_frame;
      Tom_ros.header.stamp = this->now();

      tf_broadcaster_->sendTransform(Tom_ros);


    } catch (const tf2::TransformException & ex) {
      std::cout << "   Could not find transform from " 
            << config_general_.base_frame << " to " 
            << config_general_.odom_frame << ": " << ex.what() << std::endl;
      // return res;
    }

  }

  // further ideas:
  // ask for probability (CDF) service: request AABB, return probability

  // TODO, first thoughts for robust deduction:
  // detect number of modes
  // or simpler: detect if distrubition has one mode or not (Dip test, ...)
  // or simpler: detect if the current distribution is a unimodal normal distrubution or not

  // possible potential helpful outcomes:
  // Localization category:
  // - localized
  // - not localized, because of
  //    - ambigities
  //    - not collected enough information
  // 
  
  // Fit normal distribution to data with additional scaling factor 
  // as we are fitting it to a likelihood field which just has to be 
  // proportional to an actual probability distribution

  // float weight_sum = 0.0;
  // float weight_max = 0.0;
  // for(size_t i=0; i<particle_attrs.size(); i++)
  // {
  //   weight_sum += particle_attrs[i].likelihood.mean;
  //   weight_max = std::max(weight_max, particle_attrs[i].likelihood.mean);
  // }

  // mean
  // weighted mean
  rm::Vector3f wmean = {0.0, 0.0, 0.0};
  // for(size_t i=0; i<)

  std::cout << "} // State Induction" << std::endl;
}

void RmclNode::visualize()
{
  std::shared_lock lock(data_mtx_);

  // std::cout << "-------------------" << std::endl;
  // std::cout << "{ // Visualize" << std::endl;
  const size_t n_viz_particles = std::min(config_visualization_.max_particles, n_particles_);

  // std::cout << "    - Visualize " << n_viz_particles << " particles..." << std::endl;

  // we need data on cpu
  if(data_location_ == "gpu")
  {
    // we need to have the data on cpu
    particle_cloud_->poses(0, n_viz_particles) = particle_cloud_gpu_->poses(0, n_viz_particles);
    particle_cloud_->attrs(0, n_viz_particles) = particle_cloud_gpu_->attrs(0, n_viz_particles);
  }

  // do visualizations here. read only
  viz_pcl_particles_.header.frame_id = config_general_.map_frame;
  viz_pcl_particles_.header.stamp = this->now();

  if(viz_pcl_particles_.points.size() != n_viz_particles)
  {
    viz_pcl_particles_.points.resize(n_viz_particles);
  }

  if(viz_pcl_particles_.channels.size() < 1)
  {
    sensor_msgs::msg::ChannelFloat32 channel;
    channel.name = "likelihood";
    channel.values.resize(n_viz_particles);
    viz_pcl_particles_.channels.emplace_back(channel);
  }

  if(viz_pcl_particles_.channels.size() < 2)
  {
    sensor_msgs::msg::ChannelFloat32 channel;
    channel.name = "likelihood_sigma";
    channel.values.resize(n_viz_particles);
    viz_pcl_particles_.channels.emplace_back(channel);
  }

  if(viz_pcl_particles_.channels.size() < 3)
  {
    sensor_msgs::msg::ChannelFloat32 channel;
    channel.name = "likelihood_n_meas";
    channel.values.resize(n_viz_particles);
    viz_pcl_particles_.channels.emplace_back(channel);
  }

  if(viz_pcl_particles_.channels.size() < 4)
  {
    sensor_msgs::msg::ChannelFloat32 channel;
    channel.name = "badness";
    channel.values.resize(n_viz_particles);
    viz_pcl_particles_.channels.emplace_back(channel);
  }

  for(size_t i=0; i<n_viz_particles; i++)
  {
    viz_pcl_particles_.points[i].x = particle_cloud_->poses[i].t.x;
    viz_pcl_particles_.points[i].y = particle_cloud_->poses[i].t.y;
    viz_pcl_particles_.points[i].z = particle_cloud_->poses[i].t.z;
    const rm::Gaussian1D likelihood = particle_cloud_->attrs[i].likelihood;

    float uncertainty = 1.0 - static_cast<double>(likelihood.n_meas) / static_cast<double>(MAX_N_MEAS);
    // float certainty = likelihood.sigma * uncertainty + uncertainty;

    // the more measurements
    float lowly = likelihood.mean * (likelihood.sigma * uncertainty + uncertainty);

    // the more
    viz_pcl_particles_.channels[0].values[i] = likelihood.mean;
    viz_pcl_particles_.channels[1].values[i] = likelihood.sigma;
    viz_pcl_particles_.channels[2].values[i] = likelihood.n_meas;
    viz_pcl_particles_.channels[3].values[i] = lowly;
  }

  pub_pcl_viz_->publish(viz_pcl_particles_);

  // std::cout << "} // Visualize" << std::endl;
}

} // namespace rmcl

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rmcl::RmclNode)