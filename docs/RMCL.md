# RMCL - Global 6D Localization in Meshes from 1D Range Measurements 

> Note: RMCL is under development and heavy testing. 
> Nevertheless we decided to pre-release the `v0` version in an unfinished state, because we think it can help other to see 
> whats possible. Also we hope you can test this and perhaps help us in improving / finishing the last things.

Raycasting Monte Carlo Localization (RMCL) provides a practical, real-time implementation of Monte Carlo Localization (MCL) for global robot localization, accelerated by high-performance ray tracing over triangle meshes and geometric scene graphs. MCL has a decades-long track record; our focus is making it easy to deploy and tune on real robots. The pipeline scales across diverse hardware with parameters to meet tight compute and memory budgets (including for our smallest robots). The documentation begins with hands-on usage and configuration of `rmcl_localization_node`, followed by a concise overview of the underlying concepts and design choices.

## Usage

For the following commands we assume you have the [rmcl_examples](https://github.com/amock/rmcl_examples) installed. Just follow the instructions there.

Start the RMCL node by calling

```bash
ros2 launch rmcl_examples_sim start_robot_launch.py map:=avz
```

```bash
ros2 launch rmcl_examples_rmcl rmcl_lidar3d.launch map:=avz
```

A RViz window will open and you should see something like this:

![Teaser](.resources/rmcl.gif)

Trigger global localization by calling

```bash
ros2 service call /rmcl/global_localization std_srvs/srv/Empty {}
```

Other services:
- `rmcl/initial_pose_guess`

Or draw a pose with the standard RViz tool.


## Theory

Our MCL is consists of the well known steps motion update, sensor update, and resampling. In contrast to classic MCL we update the particles point-wise. By this design we cover for motion distortion naturally.

To perform continuous updates on particles, one of our particles constists of

* State: Pose Hypothesis
* Weight: Likelihood estimate represented by an 1D Gaussian (mean, variance)
* The number of total measurements that lead to this gaussian

The following sections briefly describe what each stage of our MCL does, and presents known TODOs that remain for a fully functional and modern MCL implementation.

### Sensor Update

In contrast to classic MCL we `collect` point-wise likelihood samples over time. So for each received range measurement we perform

* a ray cast or closest point search
* evaluation: computing the error between simulated and actual measurement
* update latest likelhood estimate similar to a kalman filter update

Thus, we can streamingly update the likelihood estimate. As long as the robot don't move it should converge to the near-optimal likelihood field after a short period of time.

#### TODOs

* Check if the sensor update on GPU and CPU have the same results
* Integrate Vulkan (mid-term)

### Motion Update

Move the particles using odometry from TF and apply so called forget factors to likelihood estimates. While driving the collected likelihood might become wrong, dependend on the quality of your odometry estimation. If you, we need to forget old measurements to better adapt for errors. Therefore, we reduce the remembered number of measurements that are responsible for the current likelhood estimate by applying the forget factor (or remember factor) just to the number of total measurements.

#### TODOs

* Remove the likelihood when sliding through walls. This is only implemented in the CPU version

### Resampling

We apply impoverishment-aware resampling by a simple so-called `GladiatorResampler`. It fights against a random other enemy particle. If the other particle wins, the current particles is overriden by the other one and applied some noise.

#### TODOs

For it's simplicity the `GladiatorResampler` does a pretty good job. So it can serve as a pretty good baseline for more sophisticated resampler for development.

Possible Improvements:

* Take the approach from [MegaParticles](https://staff.aist.go.jp/k.koide/projects/icra2024_mp/) (SVGD) and use a BVH structure as an ever adapting acceleration structure around the particle set for quick nearest neighbor search. Based on some timings, the BVH in [lvr2](https://github.com/uos/lvr2) could be quick enough to do that. And it's also GPU capable. 
* Reduce the number particles smarter


