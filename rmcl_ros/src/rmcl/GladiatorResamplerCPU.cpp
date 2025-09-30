#include <rmcl_ros/rmcl/GladiatorResamplerCPU.hpp>
#include <random>
#include <memory>
#include <atomic>
#include <rmagine/math/types.h>
#include <rmagine/util/StopWatch.hpp>
#include <rmagine/util/prints.h>
#include <rmcl_ros/util/ros_helper.h>

#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>

#include <tbb/enumerable_thread_specific.h>

namespace rm = rmagine;

namespace rmcl
{

GladiatorResamplerCPU::GladiatorResamplerCPU(
  rclcpp::Node::SharedPtr node)
:node_(node)
{
  std::random_device device{};
  rand_gen_ = std::make_unique<std::mt19937>(device());
  pub_runtime_ = node_->create_publisher<std_msgs::msg::Float64>("runtime", 10);
}

void GladiatorResamplerCPU::init()
{
  updateParams();
}

void GladiatorResamplerCPU::reset()
{

}

void GladiatorResamplerCPU::updateParams()
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

// Next Resampler:
// 1. Compute stats:
//    - change of weights w.r.t. direction
// 2. Use the estimated normal distribution to resample
// KLD resampling?

struct TwofoldMixture
{
  rm::Transform mean_normal;

  // rm::Matrix<7,7, float>  C;

  double likelihood_shift; // use this to sample uniformly
};


ParticleUpdateDynamicResults GladiatorResamplerCPU::update(
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

  sw();

  // stats
  double L_sum = 0.0;
  double L_sum_sq = 0.0;
  double L_max = 0.0;
  double L_min = std::numeric_limits<double>::max();
  double L_n = static_cast<double>(particle_attrs.size());

  rm::Gaussian3D gauss_fit = rm::Gaussian3D::Identity();

  for(size_t i=0; i<particle_attrs.size(); i++)
  {
    const double v = particle_attrs[i].likelihood.mean;
    L_sum += v;
    L_sum_sq += v*v;
    L_max = std::max(L_max, v);
    L_min = std::min(L_min, v);
  }

  // const double L_mean = L_sum / L_n;
  // const double L_var  = L_sum_sq / L_n - L_mean * L_mean;
  const double L_range = L_max - L_min;
  const double L_sum_normed =  L_sum - L_min * static_cast<double>(particle_attrs.size());

  // exclude bias
  for(size_t i=0; i<particle_attrs.size(); i++)
  {
    const double v = particle_attrs[i].likelihood.mean;

    const double vn = v - L_min;

    

  }

  el = sw();

  std::cout << "!!!!!!!!!!!!!!!"  <<  std::endl;

  std::cout << "    Computing Stats: " << el << "s" << std::endl;
  std::cout << "- min, max: " << L_min << ", " << L_max << " (diff: " << L_max  -  L_min <<  ")" <<  std::endl;
  std::cout << "- L_sum: " << L_sum << ", normed: " << L_sum_normed <<  std::endl;

  

  sw();

  // std::cout << "    GAUSS FIT: " << gauss_fit.mean << std::endl;

  // std::normal_distribution<float> Nd(0.0, 1.0);
  // Best-case noises of the system
  // TODO: does this need additional parameters or can we determine 
  // these from sensor noise and/or motion noise

  // we start a tournament:
  // idx 0 vs random other  -> write to idx 0
  // idx 1 vs random other  -> write to idx 1
  // ...
  // idx N vs random other  -> write to idx N

  // Note:
  // here we have to react to a reduction of the number of particles.
  // -> Possible, since the order with this method should be random,
  //    so just cutting of the end should be OK

  const uint64_t base_seed = 123456789u;

  // Give each TBB thread its own engine with a unique seed.
  std::atomic<uint64_t> seq{0};
  tbb::enumerable_thread_specific<std::mt19937> tls_rng([&] {
      return std::mt19937(base_seed + seq.fetch_add(1, std::memory_order_relaxed));
  });

  tbb::parallel_for(
    tbb::blocked_range<size_t>(0, particle_poses_new.size(), 64),
    [&](const tbb::blocked_range<size_t>& r)
  {
    std::mt19937& rand_gen = tls_rng.local();
    std::normal_distribution<double> Nd(0.0, 1.0);
    std::uniform_int_distribution<size_t> Ud(0, particle_poses.size() - 1);

    for(size_t i = r.begin(); i != r.end(); ++i)
    {
      const size_t champion_idx = i;
      const size_t enemy_idx = Ud(rand_gen);

      const float Lc = particle_attrs[champion_idx].likelihood.mean;
      const float Le = particle_attrs[enemy_idx].likelihood.mean;

      if(Le > Lc)
      {
        const rm::Transform pose = particle_poses[enemy_idx];
        const ParticleAttributes attrs = particle_attrs[enemy_idx];

        rm::Transform pose_new = pose;
        ParticleAttributes attrs_new = attrs;

        const float noise_tx = config_.min_noise_tx;
        const float noise_ty = config_.min_noise_ty;
        const float noise_tz = config_.min_noise_tz;

        const float noise_roll  = config_.min_noise_roll;
        const float noise_pitch = config_.min_noise_pitch;
        const float noise_yaw   = config_.min_noise_yaw;

        // pre-gen random numbers
        const float Nd_tx = Nd(rand_gen);
        const float Nd_ty = Nd(rand_gen);
        const float Nd_tz = Nd(rand_gen);
        const float Nd_rx = Nd(rand_gen);
        const float Nd_ry = Nd(rand_gen);
        const float Nd_rz = Nd(rand_gen);

        pose_new.t.x += Nd_tx * noise_tx;
        pose_new.t.y += Nd_ty * noise_ty;
        pose_new.t.z += Nd_tz * noise_tz;
        rm::EulerAngles e = pose_new.R;
        e.roll += Nd_rx * noise_roll;
        e.pitch += Nd_ry * noise_pitch;
        e.yaw += Nd_rz * noise_yaw;
        pose_new.R = e;

        const rm::Transform pose_diff = ~pose * pose_new;

        // keep the likelihood, reduce number of measurements
        const float trans_dist = pose_diff.t.l2normSquared(); // in meter
        const float rot_dist = pose_diff.R.l2norm(); // in radian

        const float forget_rate_space = 1.0 - pow(1.0 - config_.likelihood_forget_per_meter, trans_dist);
        const float forget_rate_rot = 1.0 - pow(1.0 - config_.likelihood_forget_per_radian, rot_dist);
        const float forget_rate = std::max(forget_rate_space, forget_rate_rot);
        const float remember_rate = (1.0 - forget_rate);

        attrs_new.likelihood.n_meas *= remember_rate;

        particle_poses_new[champion_idx] = pose_new;
        particle_attrs_new[champion_idx] = attrs_new;
      }
      else
      {
        // champion stays champion
        particle_poses_new[champion_idx] = particle_poses[champion_idx];
        particle_attrs_new[champion_idx] = particle_attrs[champion_idx];
      }
    }
  });

  el = sw();

  // std::cout << "   Particle Stats:" << std::endl;
  // std::cout << "   - weights: " << std::endl;
  // std::cout << "      - min, max: " << L_min << ", " << L_max << std::endl;
  // std::cout << "      - mean, var: " << L_mean << "," << L_var << std::endl;
  // std::cout << "   Runtime: " << el << "s" << std::endl;

  {
    std_msgs::msg::Float64 msg;
    msg.data = el;
    pub_runtime_->publish(msg);
  }

  res.n_particles = particle_poses.size();

  return res;
}

} // namespace rmcl