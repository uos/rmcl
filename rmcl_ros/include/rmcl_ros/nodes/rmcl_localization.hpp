#ifndef RMCL_ROS_NODES_RMCL_LOCALIZATION_HPP
#define RMCL_ROS_NODES_RMCL_LOCALIZATION_HPP

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>

#include <rmagine/math/types.h>
#include <rmagine/util/prints.h>
#include <rmagine/map/EmbreeMap.hpp>
#include <rmagine/util/StopWatch.hpp>

#include <rmcl_ros/rmcl/ParticleAttributes.hpp>
#include <rmcl_ros/util/ros_helper.h>


#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>


#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/convert.h>

#include <chrono>
#include <random>

#include <shared_mutex>   // C++17
#include <mutex>


#include <rmcl_ros/rmcl/MotionUpdater.hpp>
#include <rmcl_ros/rmcl/SensorUpdater.hpp>
#include <rmcl_ros/rmcl/Resampler.hpp>

#include <rmagine/map/MapMap.hpp>


// TODO: make this optional
#include <rmagine/types/MemoryCuda.hpp>


#include <std_srvs/srv/empty.hpp>
#include <rmcl_msgs/srv/set_initial_pose.hpp>

namespace rm = rmagine;

constexpr uint32_t MAX_N_MEAS = 10000;

namespace rmcl
{
// struct BeliefStateStats
// {
//   /**
//    * @brief how similar the belief state is to a normal distribution
//    * between 0 and 1
//    * 1: is likely to be a normal distribution
//    * 0: is unlikely to be a normal distribution
//   **/ 
//   float normaly = 0.0;

//   float likelihood = 0.0;

//   // TODO: more dimensions
//   rm::Matrix3x3 cov;
//   rm::Vector3 mean;
// };

// struct RangeMeasurement
// {
//     rm::Vector      orig;
//     rm::Vector      dir;
//     float           range;
//     rm::Matrix3x3   cov;

//     inline rm::Vector3 mean() const
//     {
//       return orig + dir * range;
//     }
// };

// /**
//  * @brief this transforms a RangeMeasurement from one space to another
//  * Remember: Measurements are only comparable in the same coordinate system!
//  * Therefore, this function is very important
//  */
// RangeMeasurement transform(
//   const rm::Transform& T,
//   const RangeMeasurement& meas)
// {
//   RangeMeasurement ret;
//   const rm::Matrix3x3 R = T.R;
//   ret.dir = T.R * meas.dir;
//   ret.orig = T * meas.orig;
//   ret.range = meas.range;
//   // seems the range is not effected by a rigid transformation
//   // however, it will be effected by a transformation that include a scaling part
//   ret.cov = R * meas.cov * R.T(); // TODO: check this
//   return ret;
// }

// RangeMeasurement operator*(
//   const rm::Transform& T,
//   const RangeMeasurement& meas)
// {
//   return transform(T, meas);
// }

// float compute_z_score(
//   const rm::Gaussian1D& g,
//   float population_mean)
// {
//   constexpr float MAX_Z_SCORE = 1000000.0;

//   // z score: how likely is g to be unlikely?
//   if(g.n_meas < 1)
//   {
//     return MAX_Z_SCORE;
//   }

//   const double standard_error = g.sigma / sqrt(static_cast<double>(g.n_meas));
//   if(standard_error < 0.000001)
//   {
//     return MAX_Z_SCORE;
//   }

//   const float z_score = (g.mean - population_mean) / standard_error;
//   return std::min(z_score, MAX_Z_SCORE);
// }

// double normalCDF(double value)
// {
//    return 0.5 * erfc(-value * M_SQRT1_2);
// }

// float compute_p_value(
//   const rm::Gaussian1D& g,
//   float population_mean)
// {
//   const float z_score = compute_z_score(g, population_mean);
//   // 2 * (1 + CDF(z_score))
//   return 2.0 - erfc(-z_score * M_SQRT1_2);
// }

/**
 * good rmcl parameters are going to be guessed dependend on the beliefstate stats and the change of the belief state stats
 * some situations I want to cover:
 * 
 * 1. Surprise:
 * 
 * t0: belief state is very certain: 
 *     - unimodal
 *     - small covariances
 *     -> few particles (tracking)
 *     - high likelihood
 * 
 * Robot got kidnapped
 * 
 * t1: particles got a bad evaluation
 *      - unimodal
 *      - small covariances
 *      - low likelihood
 * 
 * difference when kidnapping the robot would be:
 * - quickly decreasing likelihood
 * 
 * How to overcome: react - increase particles, spread wider
 * 
 * (unimodal, small covariances, decreasing likelihood) -> increase number of particles
 * 
 * Note: I think, fitting a normal distribution to t1 would automalically give a "wider" spread, since all the likelihood values are low
 * 
 * -> sensor data we record are not the sensor data we expect: surprise
 * 
*/ 
struct RMCLParameters
{
  size_t num_particles = 1;
  size_t max_evaluations = 1;
  size_t num_corrections = 0;
};

/**
 * computational thoughts:
 * 
 * we have to compute different things:
 * - N particles
 * - M measurements for sensor updates
 * 
 * - M corrections (when convergeged)
 * 
 * For corrections we need enough measurements per particles
 * 
 * Evaluation is faster than correction
 * 
 * What I propose:
 * 
 * Localization status:  Computation
 * - Bad:  (N = 1000000, M = 1, corr = false)              
 * - Medium: (N = 1000, M = 1000, corr = false) 
 * - Good: (N = 500, M = 1000, corr = true)
 * 
 * I want the transition to be flowing. But trigger the correction cannot be flowing.
 * Can we enable a 
 * 
*/

template<typename MemT>
struct ParticleCloud
{
  rm::Memory<rm::Transform, MemT>        poses;
  rm::Memory<ParticleAttributes, MemT>   attrs;
};

template<typename MemT>
struct ParticleCloudView
{
  rm::MemoryView<rm::Transform, MemT>        poses;
  rm::MemoryView<ParticleAttributes, MemT>   attrs;
};

// template<typename MemT>
// std::shared_ptr<ParticleCloudView<MemT> > make_view(
//   rm::MemoryView<rm::Transform, MemT> poses,
//   rm::MemoryView<ParticleAttributes, MemT> attrs)
// {
//   return std::make_shared<ParticleCloudView<MemT> >(ParticleCloudView<MemT>{poses, attrs});
// }

// template<typename MemT>
// std::shared_ptr<ParticleCloudView<MemT> > make_view(
//   const ParticleCloud& particle_cloud)
// {
//   return make_view(particle_cloud.poses, particle_cloud.attrs);
// }

/**
 * 
 * Plan:
 * 
 * Init a lot of particles
 * 
 * 
 * 
 * 
*/
class RmclNode : public rclcpp::Node
{
public:
  explicit RmclNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  void initMemory();

  ///////////////////////////////
  // GENERAL
  ///////////////////////////////

  // 1. config
  struct {
    size_t max_particles = 100000;
    std::string base_frame;
    std::string odom_frame;
    std::string map_frame;
  } config_general_;

  // 2. members
  // this is shared to the plugins
  rm::MapMapPtr map_container_;
  std::shared_ptr<tf2_ros::TransformListener>    tf_listener_;
  std::shared_ptr<tf2_ros::Buffer>               tf_buffer_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // currently used particles for motion update, sensor update, and resampling
  size_t                                         n_particles_;

  // global data storage

  // current data location: CPU/GPU
  std::string data_location_;

  // protects particles from threading access (TODO: test if threading makes things smoother or not)
  std::shared_mutex data_mtx_;

  // CPU
  ParticleCloud<rm::RAM>* particle_cloud_;
  ParticleCloud<rm::RAM>* particle_cloud_next_;

  ParticleCloud<rm::RAM> particle_cloud_0_;
  ParticleCloud<rm::RAM> particle_cloud_1_;

  // GPU
  ParticleCloud<rm::VRAM_CUDA>* particle_cloud_gpu_;
  ParticleCloud<rm::VRAM_CUDA>* particle_cloud_gpu_next_;

  ParticleCloud<rm::VRAM_CUDA> particle_cloud_gpu_0_;
  ParticleCloud<rm::VRAM_CUDA> particle_cloud_gpu_1_;


  // 3. functions
  void updateGeneralParams();

  ///////////////////////////////////
  // INITIALIZATION
  ///////////////////////////////////
  void initSamplesUniform();
  void initSamples(const geometry_msgs::msg::PoseWithCovarianceStamped& pose);

  void updateInitializationParams();

  struct {
    std::string topic;
    size_t n_particles;
  } config_pose_initialization_;

  struct {
    std::vector<double> bb_min = {-50.0, -50.0, 0.0, 0.0, 0.0, -M_PI};
    std::vector<double> bb_max = { 50.0,  50.0, 0.0, 0.0, 0.0,  M_PI};
    size_t n_particles;
  } config_global_initialization_;

  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srv_global_initialization_;

  // rclcpp::Service<std_srvs::srv::Empty>::SharedPtr 


  ///////////////////////////////
  // MOTION UPDATE
  ///////////////////////////////

  // 1. config
  struct {
    double       rate;
    std::string  type;
    std::string  compute;
  } config_motion_update_;

  // 2. members
  rclcpp::Node::SharedPtr motion_update_node_;
  rclcpp::TimerBase::SharedPtr motion_update_timer_;
  
  std::shared_ptr<MotionUpdater<rmagine::RAM> > motion_updater_;
  std::shared_ptr<MotionUpdater<rmagine::VRAM_CUDA> > motion_updater_gpu_;

  bool motion_update_done_ = false;

  // 3. functions
  void updateMotionUpdateParams();
  void motionUpdate();
  
  ///////////////////////////////////
  // SENSOR UPDATE
  ///////////////////////////////////

  // 1. config
  struct {

    std::string type; // type of implementation

    std::string compute;

    // sensor update
    // Correspondence type
    // 0: RCC
    // 1: NN
    unsigned int correspondence_type = 0;
    
    // certainty about measurement (+pose?)
    // normally this is a value which adds up state and measurement uncertainty.
    // not implemented yet, but a wild idea:
    // - give only a parameter for measurement uncertainty here
    // - automatically detect state uncertainty from the particles
    // - add both together.
    // -> this would result in a dynamically adjusted certainty based on the current state's certainty
    float dist_sigma = 2.0;

    float likelihood_sigma = 10.0;

    unsigned int samples = 10;

    std::string topic;

  } config_sensor_update_;

  // 2. members
  rclcpp::Node::SharedPtr sensor_update_node_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcl_; // timing based on subscribtion
  std::shared_ptr<SensorUpdater<rmagine::RAM> > sensor_updater_;
  std::shared_ptr<SensorUpdater<rmagine::VRAM_CUDA> > sensor_updater_gpu_;

  bool sensor_update_done_ = false;

  // 3. functions
  void updateSensorUpdateParams();
  void sensorUpdate(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg);


  ///////////////////////////////////
  // RESAMPLING
  ///////////////////////////////////
  
  // 1. config
  struct {
    std::string type;

    std::string compute;

    double rate = 10.0;

    size_t max_induction_particles;
  
  } config_resampling_;

  // 2. members
  rclcpp::Node::SharedPtr                   resampling_node_;
  rclcpp::TimerBase::SharedPtr              resampling_timer_;
  std::shared_ptr<Resampler<rmagine::RAM> > resampler_;
  std::shared_ptr<Resampler<rmagine::VRAM_CUDA> > resampler_gpu_;

  // 3. functions
  void updateResamplingParams();
  void resampling();

  // Prepare memory. Synchronize memory between devices when computing unit is changed
  void prepareMemory(const std::string& target_device);

  ///////////////////////////////////
  // DECIDE WHERE WE ARE BASED ON BELIEF (TODO)
  ///////////////////////////////////
  // This function aims to induce the state from belief, only if possible (!)
  // It is currently called after resampling step
  // - Allow to call it from a service? -> so that another module can ask for a summary of the localization process
  // - Stream induction info on a topic independend from the resampling step? 
  void induceState(); 

  ///////////////////////////////////
  // VISUALIZATION
  ///////////////////////////////////
  
  // 1. config
  struct {
    bool enabled;
    double rate;
    size_t max_particles;
  } config_visualization_;

  // 2. members
  rclcpp::TimerBase::SharedPtr viz_timer_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pub_pcl_viz_;
  sensor_msgs::msg::PointCloud viz_pcl_particles_;

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_pose_wc_;
  // rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose;

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_pose_wc_;

  // 3. functions
  void updateVisualizationParams();
  void visualize();
};

} // namespace rmcl



#endif // RMCL_ROS_NODES_RMCL_LOCALIZATION_HPP