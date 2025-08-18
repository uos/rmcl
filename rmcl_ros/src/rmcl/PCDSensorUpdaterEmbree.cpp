
#include "rmcl_ros/rmcl/PCDSensorUpdaterEmbree.hpp"
#include <rmagine/map/EmbreeMap.hpp>
#include <rmagine/util/StopWatch.hpp>
#include <rmcl_ros/util/ros_helper.h>

#include <random>
#include <iostream>

#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>

namespace rm = rmagine;

namespace rmcl
{

float evaluate_rcc(
  const rm::EmbreeMapPtr& map,
  const RangeMeasurement& meas_m,
  const rm::Interval& sensor_range,
  const float real_hit_sim_miss_error, // in meter
  const float real_miss_sim_hit_error, // in meter
  const float real_miss_sim_miss_error
  )
{
  const bool real_hit = sensor_range.inside(meas_m.range);
  
  // ray casting correspondences metric
  RTCRayHit rayhit;
  rayhit.ray.org_x = meas_m.orig.x;
  rayhit.ray.org_y = meas_m.orig.y;
  rayhit.ray.org_z = meas_m.orig.z;
  rayhit.ray.dir_x = meas_m.dir.x;
  rayhit.ray.dir_y = meas_m.dir.y;
  rayhit.ray.dir_z = meas_m.dir.z;
  rayhit.ray.tnear = 0;
  rayhit.ray.tfar = std::numeric_limits<float>::infinity();
  rayhit.ray.mask = -1;
  rayhit.ray.flags = 0;
  rayhit.hit.geomID = RTC_INVALID_GEOMETRY_ID;
  rayhit.hit.instID[0] = RTC_INVALID_GEOMETRY_ID;

  rtcIntersect1(map->scene->handle(), &rayhit);

  const float sim_range = rayhit.ray.tfar;
  const bool sim_hit = (rayhit.hit.geomID != RTC_INVALID_GEOMETRY_ID && sim_range > sensor_range.min);

  // deterimine error
  float error = 0.0;
  if(sim_hit)
  {
    // sim hit
    if(real_hit)
    {
      // sim hit + real hit
      rm::Vector nint_m {
            rayhit.hit.Ng_x,
            rayhit.hit.Ng_y,
            rayhit.hit.Ng_z
        };
    
      // point to plane distance
      const rm::Vector preal_m = meas_m.mean();
      const rm::Vector pint_m = meas_m.orig + meas_m.dir * sim_range;

      const float signed_plane_dist = (pint_m - preal_m).dot(nint_m);
      error = abs(signed_plane_dist);
    } else {
      // hit in simulation but not in reality -> bad
      error = real_miss_sim_hit_error;
    }
  } else {
    // sim miss
    if(real_hit)
    {
      // hit in reality but not in sim -> bad
      error = real_hit_sim_miss_error;
    } else {
      // not hit in reality and not hit in simulation -> good (but how good?)
      error = real_miss_sim_miss_error;
    }
  }

  return error;
}

float evaluate_cpc(
  const rm::EmbreeMapPtr& map,
  const RangeMeasurement& meas_m)
{
  // distance to surface
  const rm::EmbreeClosestPointResult res = map->closestPoint(meas_m.mean());
  return res.d;
}

PCDSensorUpdaterEmbree::PCDSensorUpdaterEmbree(
  rm::MapMapPtr map_container,
  rclcpp::Node::SharedPtr node,
  std::shared_ptr<tf2_ros::Buffer> tf_buffer)
:map_container_(map_container)
,node_(node)
,tf_buffer_(tf_buffer)
{
  pub_runtime_ = node_->create_publisher<std_msgs::msg::Float64>("runtime", 10);
}

void PCDSensorUpdaterEmbree::init()
{
  std::cout << "   Init PCDSensorUpdaterEmbree" << std::endl;
  base_frame_ = rmcl::get_parameter(node_, "base_frame", "base_link");

  updateParams();
  loadMap();
}

void PCDSensorUpdaterEmbree::reset()
{

}

void PCDSensorUpdaterEmbree::updateParams()
{
  config_.samples = rmcl::get_parameter(node_, "~samples", 100);
  config_.correspondence_type = rmcl::get_parameter(node_, "~correspondence_type", 0);
  config_.dist_sigma = rmcl::get_parameter(node_, "~dist_sigma", 2.0);
  
  config_.real_hit_sim_miss_error = rmcl::get_parameter(node_, "~real_hit_sim_miss_error", 100.0);
  config_.real_miss_sim_hit_error = rmcl::get_parameter(node_, "~real_miss_sim_hit_error", 100.0);
  config_.real_miss_sim_miss_error = rmcl::get_parameter(node_, "~real_miss_sim_miss_error", 0.0);

  config_.sensor_range.min = rmcl::get_parameter(node_, "~sensor_range_min", 0.05);
  config_.sensor_range.max = rmcl::get_parameter(node_, "~sensor_range_max", 80.0);
}

void PCDSensorUpdaterEmbree::loadMap()
{
  std::cout << "   Loading map!" << std::endl;
  auto map_param_opt = rmcl::get_parameter(node_, "~map");
  if(map_param_opt)
  {
    std::string map_name = map_param_opt->as_string();
    std::string map_key = map_name + ".embree";

    if(map_container_->find(map_key) == map_container_->end())
    {
      // we need to newly load the map
      std::stringstream ss;
      ss << "map_server." << map_name << ".source";
      std::string source_param_name = ss.str();
      std::string source = rmcl::get_parameter(node_, source_param_name)->as_string();

      std::cout << "   Loading '" << map_name << "' from '" << source << "'" << std::endl;

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
      std::cout << "   Using cached map!" << std::endl;
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
    std::stringstream error_msg;
    error_msg << "ERROR: Map not specified for sensor update!";
    std::cout << error_msg.str() << std::endl;
    throw std::runtime_error(error_msg.str());
  }
}

/**
 * sensorUpdate for a single 1D measurement
 * 
 * Problem: state_uncertainty is multimodal, however, it converges against one mode (in the best case)
 * The particle belongs to one mode. Therefore I assume its OK. But I have no mathematical proof
 */
ParticleAttributes PCDSensorUpdaterEmbree::sensorUpdate(
  const rm::Transform& Tsm,
  const RangeMeasurement& meas_s,
  const ParticleAttributes& particle_attr
) const
{
  // params
  const float sigma_dist = config_.dist_sigma;
  const float sigma_dist_quad = sigma_dist * sigma_dist;

  const RangeMeasurement meas_m = Tsm * meas_s;

  // determine error based on method
  float error = 0.0;
  if(config_.correspondence_type == 0) 
  {
    error = evaluate_rcc(map_, meas_m, 
      config_.sensor_range,
      config_.real_hit_sim_miss_error,
      config_.real_miss_sim_hit_error,
      config_.real_miss_sim_miss_error);
  }
  else if(config_.correspondence_type == 1)
  { 
    error = evaluate_cpc(map_, meas_m);
  }

  const float eval = exp(-(error * error) / sigma_dist_quad / 2) / (sqrt(2 * sigma_dist_quad * M_PI));
  // famous AMCL magic formular
  // const float eval_eval_eval = eval * eval * eval;
  // in MCL code: sum the squared evals
  // - we have to do everything incrementally

  ParticleAttributes particle_attr_new = particle_attr;

  rm::Gaussian1D meas;
  meas.mean = eval;
  meas.sigma = 0.0;
  meas.n_meas = 1;

  particle_attr_new.likelihood += meas;
  particle_attr_new.likelihood.n_meas = std::min(particle_attr_new.likelihood.n_meas, MAX_N_MEAS);

  return particle_attr_new;
}


ParticleUpdateResults PCDSensorUpdaterEmbree::update(
  rmagine::MemoryView<rmagine::Transform, rmagine::RAM> particle_poses,
  rmagine::MemoryView<ParticleAttributes, rmagine::RAM> particle_attrs,
  const ParticleUpdateConfig& config)
{
  updateParams();
  ParticleUpdateResults res;

  const auto scene = map_->scene;

  // time interpolation if time field in data exists
  rm::Transform Tsb;

  try {
    geometry_msgs::msg::TransformStamped T = tf_buffer_->lookupTransform(
      base_frame_, // to
      input_->header.frame_id, // from
      input_->header.stamp,
      tf2::durationFromSec(0.3));
    Tsb.t.x = T.transform.translation.x;
    Tsb.t.y = T.transform.translation.y;
    Tsb.t.z = T.transform.translation.z;
    Tsb.R.x = T.transform.rotation.x;
    Tsb.R.y = T.transform.rotation.y;
    Tsb.R.z = T.transform.rotation.z;
    Tsb.R.w = T.transform.rotation.w;  
  } catch (const tf2::TransformException & ex) {
    RCLCPP_INFO_STREAM(node_->get_logger(), "   Could not find transform from " 
      << input_->header.frame_id << " to " << base_frame_ << ". Error: " << ex.what());
    return res;
  }

  static std::random_device rd;  // a seed source for the random number engine
  static std::mt19937 gen(rd()); // mersenne_twister_engine seeded with rd()
  static std::uniform_int_distribution<size_t> dist(0, input_->width * input_->height - 1);

  // std::cout << "- USING THREADS: " << omp_get_max_threads() << std::endl;
  // std::cout << "- particles: " << particle_poses.size() << std::endl;
  // std::cout << "- beams: " << config_.samples << std::endl;
  // std::cout << "- correspondence type: " << config_.correspondence_type << std::endl;
  // std::cout << "- dist sigma: " << config_.dist_sigma << std::endl;

  double el;
  rm::StopWatch sw;

  sw();
  for(size_t sample_id = 0; sample_id < config_.samples; sample_id++)
  {
    int max_tries = 100;
    bool is_valid = false;
    size_t random_point_id = 0;

    rm::Vector3f random_point;
    for(int i=0; i<max_tries && !is_valid; i++)
    {
      random_point_id = dist(gen);
      const uint8_t* random_point_ptr = &input_->data[random_point_id * input_->point_step];
      // no risk no fun
      random_point = *reinterpret_cast<const rm::Vector3f*>(random_point_ptr);
      is_valid = (random_point.x == random_point.x && random_point.y == random_point.y && random_point.z == random_point.z);
    }

    if(!is_valid)
    {
      // TODO: do this better
      std::cout << "   Point invalid" << std::endl;
      return res;
    }

    // define range measurement in sensor coords
    RangeMeasurement meas_s;
    { // fill measurement struct
      meas_s.orig = {0.0, 0.0, 0.0};
      meas_s.dir = random_point.normalize();
      meas_s.range = random_point.l2norm();
      
      // covariance in ray coords (very local coors)
      rm::Matrix3x3 cov_r = rm::Matrix3x3::Identity() * 0.1;

      // TODO!: Transform from ray to sensor coords
      rm::Transform Trs = rm::Transform::Identity();
      const rm::Matrix3x3 Rrs = Trs.R;
      meas_s.cov = Rrs * cov_r * Rrs.T();
    }

    // compute partial sensor update for all particles
    tbb::parallel_for( 
      tbb::blocked_range<size_t>(0, particle_poses.size(), 128),
      [&](const tbb::blocked_range<size_t>& r)
    {
      for(size_t i=r.begin(); i<r.end(); ++i)
      {
        // read
        const rm::Transform Tbm = particle_poses[i];
        const rm::Transform Tsm = Tbm * Tsb;
        particle_attrs[i] = sensorUpdate(Tsm, meas_s, particle_attrs[i]);
      }
    });
  }
  el = sw();

  {
    std_msgs::msg::Float64 msg;
    msg.data = el;
    pub_runtime_->publish(msg);
  }
  
  return res;
}

} // namespace rmcl