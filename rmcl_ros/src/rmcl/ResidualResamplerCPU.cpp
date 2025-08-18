#include <rmcl_ros/rmcl/ResidualResamplerCPU.hpp>
#include <random>
#include <memory>
#include <rmagine/math/types.h>
#include <rmagine/util/StopWatch.hpp>
#include <rmcl_ros/util/ros_helper.h>

#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>

namespace rm = rmagine;

namespace rmcl
{

ResidualResamplerCPU::ResidualResamplerCPU(
  rclcpp::Node::SharedPtr node)
:node_(node)
{
  std::random_device device{};
  rand_gen_ = std::make_unique<std::mt19937>(device());
  pub_runtime_ = node_->create_publisher<std_msgs::msg::Float64>("runtime", 10);
}

void ResidualResamplerCPU::init()
{
  updateParams();
}

void ResidualResamplerCPU::reset()
{

}

void ResidualResamplerCPU::updateParams()
{
  config_.min_noise_tx = rmcl::get_parameter(node_, "~min_noise_tx", 0.03);
  config_.min_noise_ty = rmcl::get_parameter(node_, "~min_noise_ty", 0.03);
  config_.min_noise_tz = rmcl::get_parameter(node_, "~min_noise_tz", 0.0);
  
  config_.min_noise_roll  = rmcl::get_parameter(node_, "~min_noise_roll", 0.0);
  config_.min_noise_pitch = rmcl::get_parameter(node_, "~min_noise_pitch", 0.0);
  config_.min_noise_yaw   = rmcl::get_parameter(node_, "~min_noise_yaw", 0.01);

  config_.likelihood_forget_per_meter = rmcl::get_parameter(node_, "~likelihood_forget_per_meter", 0.3);
  config_.likelihood_forget_per_radian = rmcl::get_parameter(node_, "~likelihood_forget_per_radian", 0.2);
}

ParticleUpdateDynamicResults ResidualResamplerCPU::update(
  const rmagine::MemoryView<rmagine::Transform, rmagine::RAM> particle_poses,
  const rmagine::MemoryView<ParticleAttributes, rmagine::RAM> particle_attrs,
  rmagine::MemoryView<rmagine::Transform, rmagine::RAM> particle_poses_new,
  rmagine::MemoryView<ParticleAttributes, rmagine::RAM> particle_attrs_new,
  const ParticleUpdateDynamicConfig& config)
{
  updateParams();

  rm::StopWatch sw;
  double el;

  ParticleUpdateDynamicResults res;

  float weight_sum = 0.0;
  float weight_max = 0.0;

  for(size_t i=0; i<particle_attrs.size(); i++)
  {
    weight_sum += particle_attrs[i].likelihood.mean;
    weight_max = std::max(weight_max, particle_attrs[i].likelihood.mean);
  }

  std::cout << "   Particle Stats:" << std::endl;
  std::cout << "   - weight sum: " << weight_sum << std::endl;
  std::cout << "   - weight max: " << weight_max << std::endl;

  std::uniform_int_distribution<size_t> uniform_distribution(0, particle_poses.size() - 1);
  

  // Best-case noises of the system
  // TODO: does this need additional parameters or can we determine 
  // these from sensor noise and/or motion noise

  // How can we thread this algorithm?

  std::vector<std::thread> workers;


  

  sw();
  size_t insertion_idx = 0;
  size_t loop_iterations = 0;
  
  while(insertion_idx < particle_poses_new.size())
  {
    const size_t random_index = uniform_distribution(*rand_gen_);

    // sample around this pose
    const rm::Transform pose =       particle_poses[random_index];
    const ParticleAttributes attrs = particle_attrs[random_index];

    const float L = attrs.likelihood.mean;
    const float L_sum_normed = L / weight_sum; // all L_normed are in sum 1; in [0, 1]
    const float L_max_normed = L / weight_max; // L / L_max = L_normed2; in [0, 1]

    const size_t n_expected_insertions = static_cast<double>(L_sum_normed) * static_cast<double>(particle_poses_new.size());
    const size_t n_insertions_left = particle_poses_new.size() - insertion_idx;

    const size_t n_insertions = 
                  (n_expected_insertions <= n_insertions_left) ? 
                  n_expected_insertions : n_insertions_left;

    // std::cout << n_insertions << " Left: " << n_insertions_left << std::endl;

    if(n_insertions > 0 && (insertion_idx + n_insertions - 1) >= particle_poses_new.size()) [[unlikely]] 
    {
      std::cout << "insertion_idx + n_insertions >= particle_poses_new.size(): " << insertion_idx << " + " << n_insertions << " >= " << particle_poses_new.size() << std::endl;
      throw std::runtime_error("ResidualResamplerCPU: Fix implementation!");
    }

    auto residual_fill = [=, &particle_poses_new, &particle_attrs_new]()
    {
      std::normal_distribution<float> Nd(0.0, 1.0); // mean = 0, stddev = 1

      for (size_t inner_idx = 0; inner_idx < n_insertions; ++inner_idx)
      {
        rm::Transform pose_new = pose;
        ParticleAttributes attrs_new = attrs;

        // add noise, based on 
        // - a-priori noise. how to get it here?
        // - likelihood? is this correct?
        // TODO

        const float noise_tx = config_.min_noise_tx / L_max_normed;
        const float noise_ty = config_.min_noise_ty / L_max_normed;
        const float noise_tz = config_.min_noise_tz / L_max_normed;

        const float noise_roll  = config_.min_noise_roll  / L_max_normed;
        const float noise_pitch = config_.min_noise_pitch / L_max_normed;
        const float noise_yaw   = config_.min_noise_yaw   / L_max_normed;

        pose_new.t.x += Nd(*rand_gen_) * noise_tx;
        pose_new.t.y += Nd(*rand_gen_) * noise_ty;
        pose_new.t.z += Nd(*rand_gen_) * noise_tz;
        rm::EulerAngles e = pose_new.R;
        e.roll += Nd(*rand_gen_) * noise_roll;
        e.pitch += Nd(*rand_gen_) * noise_pitch;
        e.yaw += Nd(*rand_gen_) * noise_yaw;
        pose_new.R = e;

        const rm::Transform pose_diff = ~pose * pose_new;

        // keep the likelihood, reduce number of measurements
        const float trans_dist = pose_diff.t.l2normSquared(); // in meter
        const float rot_dist = pose_diff.R.l2norm(); // in radian
        const float reduction_factor = pow(config_.likelihood_forget_per_meter, trans_dist) 
                                    * pow(config_.likelihood_forget_per_radian, rot_dist);
        attrs_new.likelihood.n_meas *= reduction_factor;
        // TODO better:
        // ultimately the new likelihood depends on neigboring particles. take the information of the closest n particles
        // LSH again?

        particle_poses_new[insertion_idx + inner_idx] = pose_new;
        particle_attrs_new[insertion_idx + inner_idx] = attrs_new;
      }
    }; // residual_fill

    if(n_insertions > 10)
    {
      workers.emplace_back([=]{
        residual_fill();
      });
    } else if(n_insertions > 0) {
      residual_fill();
    }

    // we can do this asynchronously
    // residual_fill();

    insertion_idx += n_insertions;
    loop_iterations++;

    // if(loop_iterations % 1000 == 0)
    // {
    //   std::cout << "Insertion Idx: " << insertion_idx << std::endl; 
    // }
  }

  for(auto& w: workers){ 
    w.join();
  }

  el = sw();

  std::cout << "Waited for " << workers.size() << " large blocks" << std::endl;

  std::cout << "   - runtime: " << el << "s" << std::endl;

  {
    std_msgs::msg::Float64 msg;
    msg.data = el;
    pub_runtime_->publish(msg);
  }

  res.n_particles = insertion_idx;

  return res;
}

} // namespace rmcl