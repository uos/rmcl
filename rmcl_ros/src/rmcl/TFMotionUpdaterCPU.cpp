#include "rmcl_ros/rmcl/TFMotionUpdaterCPU.hpp"
#include <iostream>
#include <rmagine/map/EmbreeMap.hpp>
#include <sstream>
#include <rmagine/util/StopWatch.hpp>

#include <rmcl_ros/util/ros_helper.h>

#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>

namespace rm = rmagine;

namespace rmcl
{

bool collision_in_between(
  const rm::EmbreeMapPtr& map,
  const rm::Vector3& p1,
  const rm::Vector3& p2)
{
  rm::Vector3 vec = p2 - p1;
  const float length = vec.l2norm();

  if(length < 0.00001)
  {
    return false;
  }

  vec /= length;

  // ray casting correspondences metric
  RTCRayHit rayhit;
  rayhit.ray.org_x = p1.x;
  rayhit.ray.org_y = p1.y;
  rayhit.ray.org_z = p1.z;
  rayhit.ray.dir_x = vec.x;
  rayhit.ray.dir_y = vec.y;
  rayhit.ray.dir_z = vec.z;
  rayhit.ray.tnear = 0;
  rayhit.ray.tfar = length;
  rayhit.ray.mask = -1;
  rayhit.ray.flags = 0;
  rayhit.hit.geomID = RTC_INVALID_GEOMETRY_ID;
  rayhit.hit.instID[0] = RTC_INVALID_GEOMETRY_ID;

  rtcIntersect1(map->scene->handle(), &rayhit);

  return (rayhit.hit.geomID != RTC_INVALID_GEOMETRY_ID);
}

void TFMotionUpdaterCPU::init()
{
  std::cout << "   TFMotionUpdaterCPU::INIT!" << std::endl;
  // motion update params
  updateParams();
  loadMap();
}

void TFMotionUpdaterCPU::loadMap()
{
  if(auto map_param_opt = rmcl::get_parameter(node_, "~map"))
  {
    std::string map_name = map_param_opt->as_string();
    std::string map_key = map_name + ".embree";
    if(map_container_->find(map_key) == map_container_->end())
    {
      // we need to newly load the map
      std::stringstream ss;
      ss << "map_server." << map_name << ".source";
      std::string source_param_name = ss.str();

      std::cout << "   Loading parameter '" << source_param_name << "'" << std::endl;
      std::string source = rmcl::get_parameter(node_, source_param_name)->as_string();

      // TODO: introduce different sources. like https://github.com/ros/resource_retriever
      // default is just a file path
      rm::EmbreeMapPtr map = rm::import_embree_map(source);

      if(!map)
      {
        std::stringstream error_msg;
        error_msg << "ERROR: Could not load map from '" << source << "'";
        std::cout << error_msg.str() << std::endl;
        throw std::runtime_error(error_msg.str());
      }

      // insert map into container
      // it gets available to others now
      map_container_->insert({map_key, map});
    } else {
      std::cout << "   Using cached map" << std::endl;
    }

    map_ = std::dynamic_pointer_cast<rm::EmbreeMap>(map_container_->at(map_key));
    if(!map_)
    {
      // ERROR
      std::stringstream error_msg;
      error_msg << "ERROR: Could not load map '" << map_name << "' from map container";
      std::cout << error_msg.str() << std::endl;
      throw std::runtime_error(error_msg.str());
    }
  } else {
    std::cout << "   Motion Update: no map found. disabling collision" << std::endl;
  }
}

void TFMotionUpdaterCPU::updateParams()
{
  // config_.weight_noise = rmcl::get_parameter(node_, "~weight_noise", 0.3);
  config_.forget_rate = rmcl::get_parameter(node_, "~forget_rate", 0.5);
  config_.forget_rate_per_second = rmcl::get_parameter(node_, "~forget_rate_per_second", 0.1);
}

ParticleUpdateResults TFMotionUpdaterCPU::update(
  rmagine::MemoryView<rmagine::Transform, rm::RAM> particle_poses, 
  rmagine::MemoryView<ParticleAttributes, rm::RAM> particle_attrs,
  const ParticleUpdateConfig& config)
{
  updateParams();
  // std::cout << "-----------------------" << std::endl;
  // std::cout << "Motion Update (TF, CPU)" << std::endl;

  ParticleUpdateResults res;

  rm::Transform T_bnew_o;
  rclcpp::Time T_bnew_o_stamp;

  try {
    geometry_msgs::msg::TransformStamped T = tf_buffer_->lookupTransform(
      odom_frame_, // to
      base_frame_, // from
      node_->now(), // TODO: is this right?
      tf2::durationFromSec(0.3));

    T_bnew_o_stamp = T.header.stamp;

    T_bnew_o.t.x = T.transform.translation.x;
    T_bnew_o.t.y = T.transform.translation.y;
    T_bnew_o.t.z = T.transform.translation.z;
    T_bnew_o.R.x = T.transform.rotation.x;
    T_bnew_o.R.y = T.transform.rotation.y;
    T_bnew_o.R.z = T.transform.rotation.z;
    T_bnew_o.R.w = T.transform.rotation.w;
  } catch (const tf2::TransformException & ex) {
    std::cout << "   Could not find transform from " << base_frame_ << " to " << odom_frame_ << ": " << ex.what() << std::endl;
    return res;
  }

  if(!T_bold_o_stamp_)
  {
    // also after reset
    T_bold_o_ = T_bnew_o;
    T_bold_o_stamp_ = T_bnew_o_stamp;
    return res;
  }

  double dt = (T_bnew_o_stamp - *T_bold_o_stamp_).seconds();
  if(dt <= 0.0000001)
  {
    return res;
  }

  const rm::Transform T_bnew_bold = ~T_bold_o_ * T_bnew_o;
  double dist_travelled = T_bnew_bold.t.l2norm();
  // uint32_t forget_measurements = 100 * dist_travelled; // make this dependend from a parameter
  
  // std::cout << "- Dist travelled: " << dist_travelled << "m" << std::endl;
  // std::cout << "- Forget Measurements: " << forget_measurements << std::endl;
  // std::cout << "- Current Measurements: " << particle_attrs[0].likelihood.n_meas << std::endl;

  // TODO: rotation?

  // In other words: T_bnew_bold is the delta in bold coordinate system
  // Particle: T_bold_m -> T_bnew_m. Using T_bnew_bold
  // T_bnew_m = T_bold_m * T_bnew_bold
  // mutex?

  // forget_rate per meter to absolute -> exponential

  std::cout << "   config forgets:" << std::endl;
  std::cout << "   - space: " << config_.forget_rate << std::endl;
  std::cout << "   - time: " << config_.forget_rate_per_second << std::endl;

  std::cout << "   time & space:" << std::endl;
  std::cout << "   - space: " << dist_travelled << std::endl;
  std::cout << "   - time: " << dt << std::endl;

  const double forget_rate_space = 1.0 - pow(1.0 - config_.forget_rate, dist_travelled);
  const double forget_rate_time = 1.0 - pow(1.0 - config_.forget_rate_per_second, dt);
  const double forget_rate = forget_rate_space * forget_rate_time;

  std::cout << "   applied forgets:" << std::endl;
  std::cout << "   - space: " << forget_rate_space << std::endl;
  std::cout << "   - time: " << forget_rate_time << std::endl;
  std::cout << "   - combined: " << forget_rate << std::endl;

  rm::StopWatch sw;
  double el;

  sw();
  tbb::parallel_for( 
    tbb::blocked_range<size_t>(0, particle_poses.size(), 128),
    [&](const tbb::blocked_range<size_t>& r)
  {
    for(size_t i=r.begin(); i<r.end(); ++i)
    {
      // read
      const rm::Transform pose_old = particle_poses[i];
      ParticleAttributes attr = particle_attrs[i];

      // update poses
      const rm::Transform pose_new = pose_old * T_bnew_bold;

      // equals:
      // T_bnew_map = T_bold_map * T_bnew_bold
      // -- inverse
      // ~T_bnew_map = ~T_bnew_bold * ~T_bold_map
      // T_map_bnew = T_bold_bnew * T_map_bold

      // forget things to make space for new
      attr.likelihood.n_meas -= forget_rate * attr.likelihood.n_meas;

      // check collision, if enabled
      if(map_)
      {
        // check if way from pose_old to pose_new was passing a wall
        // do it by simply checking the ray (assuming high frequency of motion estimation)
        if(collision_in_between(map_, pose_old.t, pose_new.t))
        {
          // set likelihood to lowest value, with highest certainty
          attr.likelihood.mean = 0.0;
          attr.likelihood.sigma = 0.0;
          attr.likelihood.n_meas = MAX_N_MEAS;
        }
      }

      // set changes
      particle_poses[i] = pose_new;
      particle_attrs[i] = attr;
    }
  });
  el = sw();

  // std::cout << "- New Measurements: " << particle_attrs[0].likelihood.n_meas << std::endl;
  // std::cout << "- Runtime: " << el << "s" << std::endl;

  // if(el > (1.0 / config_motion_update_.rate))
  // {
  //   std::cout << "WARNING: Could not hold required update rate! maximum possible: " << 1.0 / el << std::endl; 
  // }

  T_bold_o_ = T_bnew_o;
  T_bold_o_stamp_ = T_bnew_o_stamp;

  return res;
}

} // namespace rmcl