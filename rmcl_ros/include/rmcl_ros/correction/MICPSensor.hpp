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
#include <rmagine/math/statistics.h>

#include <rmcl_ros/correction/Correspondences.hpp>




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
  void fetchTF();

  inline std::mutex& mutex()
  {
    return data_correction_mutex_;
  }

  virtual void findCorrespondences() = 0;

  virtual bool correspondencesOutdated() const = 0;

  virtual rmagine::CrossStatistics computeCrossStatistics(
    const rmagine::Transform& Tpre
  ) const = 0;

  // name of the sensor
  std::string name;

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
  
  
  rclcpp::Time dataset_stamp_;
  std::mutex data_correction_mutex_;

  // ROS
  rclcpp::Node::SharedPtr nh_;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // This is called as soon as data was received and pre-processed
  std::function<void(MICPSensorBase*)> on_data_received;
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
    const rmagine::Transform& T_bnew_bold // Tpre_b
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
    const rm::CrossStatistics stats_s = correspondences_->computeCrossStatistics(T_snew_sold);
    // transform CrossStatistics of every sensor to base frame
    const rm::CrossStatistics stats_b = Tsb * stats_s;
    return stats_b;
  }

  std::shared_ptr<Correspondences_<MemT> > correspondences_;

};


// using MICPSensor = MICPSensor_<rmagine::RAM>;

// using MICPSensorPtr = std::shared_ptr<MICPSensor>;

} // namespace rmcl


#endif // RMCL_CORRECTION_MICP_SENSOR_HPP