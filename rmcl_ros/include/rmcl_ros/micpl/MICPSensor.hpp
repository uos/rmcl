#ifndef RMCL_CORRECTION_MICP_SENSOR_HPP
#define RMCL_CORRECTION_MICP_SENSOR_HPP

#include <rclcpp/rclcpp.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/create_timer_ros.h>

#include <memory>
#include <string>
#include <thread>
#include <mutex>

#include <rmagine/math/types/CrossStatistics.hpp>
#include <rmagine/types/Memory.hpp>

#include <rmcl/registration/Correspondences.hpp>

#include <visualization_msgs/msg/marker.hpp>


namespace rmcl
{

class MICPSensorBase
{
public:
  MICPSensorBase(rclcpp::Node::SharedPtr nh);
  virtual ~MICPSensorBase();

  // Pipeline: load data -> search for correspondences -> update statistics
  // (later from base, for each sensor: merge statistics and compute pose corrections)

  // void setTbm(const rmagine::Transform& Tbm);
  void setTom(const rmagine::Transform& Tom);

  /**
   * Fetch TF chain. except for odom->map (this is estimated or has to be provided from extern)
   */
  bool fetchTF(const rclcpp::Time stamp);

  virtual void drawCorrespondences() = 0;

  inline std::mutex& mutex()
  {
    return data_correction_mutex_;
  }

  virtual void findCorrespondences() = 0;

  virtual bool correspondencesOutdated() const = 0;

  virtual rmagine::CrossStatistics computeCrossStatistics(
    const rmagine::Transform& Tpre,
    double convergence_progress = 0.0
  ) const = 0;

  virtual void connectToTopic(const std::string& topic_name) = 0;

  virtual void getDataFromParameters() = 0;

  inline void asyncSpin()
  {
    exec_thread_ = std::thread([this]() {
      // main function
      exec_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
      exec_->add_callback_group(cb_group_, nh_->get_node_base_interface());
      exec_->spin();
    });
  }

  // name of the sensor
  std::string name;
  bool static_dataset = false;

  /**
   * This drops performance!
   */
  bool enable_visualizations = false;

  // transform chain from sensor -> base -> odom -> map

  // keep this up to date
  rmagine::Transform Tsb;
  rclcpp::Time Tsb_stamp;
  rmagine::Transform Tbo;
  rclcpp::Time Tbo_stamp;

  rmagine::Transform Tom; // this is set from externally
  rclcpp::Time Tom_stamp;

  std::string map_frame = "map";
  std::string odom_frame = "odom";
  std::string base_frame = "base_footprint";
  std::string sensor_frame = "velodyne";

  bool first_message_received = false; 

  // weight multiplier
  double merge_weight_multiplier = 1.0;
  
  rclcpp::Time dataset_stamp_;
  std::mutex data_correction_mutex_;

  // stats
  size_t total_dataset_measurements;
  size_t valid_dataset_measurements;

  // This is called as soon as data was received and pre-processed
  std::function<void(MICPSensorBase*)> on_data_received;

protected:

  // ROS
  rclcpp::Node::SharedPtr nh_;

  // TF
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  // Visualization
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr 
    correspondence_viz_pub_;

  rclcpp::CallbackGroup::SharedPtr cb_group_;
  std::shared_ptr<rclcpp::Executor> exec_;
  std::thread exec_thread_;
};

using MICPSensorPtr = std::shared_ptr<MICPSensorBase>;

template<typename MemT>
class MICPSensor_
: public MICPSensorBase
{
public:
  MICPSensor_(rclcpp::Node::SharedPtr nh)
  :MICPSensorBase(nh)
  {
    
  }

  virtual void findCorrespondences() 
  {
    const rmagine::Transform Tbm = Tom * Tbo;
    correspondences_->find(Tbm);
    correspondences_->outdated = false;
  }

  virtual bool correspondencesOutdated() const 
  {
    return correspondences_->outdated;
  }

  virtual rmagine::CrossStatistics computeCrossStatistics(
    const rmagine::Transform& T_bnew_bold, // Tpre_b
    double convergence_progress = 0.0
  ) const
  { 
    // this is what we want to optimize: 
    // find a transformation from a new base frame to the old base frame that optimizes the alignment
    
    // bnew --- T_bnew_bold --> bold    : base frame (shared frame of all robot sensors)
    //  ^                        ^
    //  |                        |
    // Tsb                      Tsb
    //  |                        |
    // snew --- T_snew_sold --> sold    : sensor frames
    // 
    // Fig 1: How to transform delta transforms

    namespace rm = rmagine;

    // reduce correspondences_ to C
    const rm::Transform T_snew_sold = ~Tsb * T_bnew_bold * Tsb;
    const rm::CrossStatistics stats_s = correspondences_->computeCrossStatistics(
      T_snew_sold, convergence_progress);
    // transform CrossStatistics of every sensor to base frame
    const rm::CrossStatistics stats_b = Tsb * stats_s;
    return stats_b;
  }

  std::shared_ptr<Correspondences_<MemT> > correspondences_;

  // this is a temporary storage for incoming topic data
  rmagine::PointCloud_<rmagine::RAM> dataset_cpu_;
};

} // namespace rmcl


#endif // RMCL_CORRECTION_MICP_SENSOR_HPP