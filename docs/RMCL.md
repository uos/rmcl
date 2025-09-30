# RMCL - Global 6D Localization in Meshes from 1D Range Measurements

> Note: RMCL is under development and undergoing heavy testing.
> Nevertheless, we decided to pre-release version `v0` in an unfinished state because we believe it can already help others see what’s possible. We also hope you can test it and perhaps help us improve and finalize the remaining pieces.

Raycasting Monte Carlo Localization (RMCL) provides a practical, real-time implementation of Monte Carlo Localization (MCL) for global robot localization, accelerated by high-performance ray tracing over triangle meshes and geometric scene graphs. MCL has a decades-long track record; our focus is making it easy to deploy and tune on real robots. The pipeline scales across diverse hardware, with parameters to meet tight compute and memory budgets (including for our smallest robots). The documentation begins with hands-on usage and configuration of `rmcl_localization_node`, followed by a concise overview of the underlying concepts and design choices.

## Usage

For the following commands, we assume you have the [rmcl_examples](https://github.com/amock/rmcl_examples) installed. Just follow the instructions there.

Start the RMCL node by running:

```bash
ros2 launch rmcl_examples_sim start_robot_launch.py map:=avz
```

```bash
ros2 launch rmcl_examples rmcl_rmcl.launch map:=avz
```

An RViz window will open, and you should see something like this:

![Teaser](../.resources/rmcl.gif)

Trigger global localization by calling:

```bash
ros2 service call /rmcl/global_localization std_srvs/srv/Empty {}
```

Other services:

* `rmcl/initial_pose_guess`

Or draw a pose with the standard RViz tool. See the [rmcl_examples](https://github.com/amock/rmcl_examples) repository for more examples.

## Theory

Our MCL consists of the well-known steps: motion update, sensor update, and resampling. In contrast to classic MCL, we update the particles point-wise. This design naturally accounts for motion distortion. To perform continuous updates on particles, each particle consists of:

* State: Pose hypothesis
* Weight: Likelihood estimate represented by a 1D Gaussian (mean, variance)
* The total number of measurements that contributed to this Gaussian

The following sections briefly describe what each stage of our MCL does, and present the remaining TODOs for a fully functional and modern implementation.

### Sensor Update

In contrast to classic MCL, we "collect" point-wise likelihood samples over time. For each received range measurement we perform:

* a ray cast or closest point search
* evaluation: computing the error between simulated and actual measurement
* updating the latest likelihood estimate, similar to a Kalman filter update

Thus, we can update the likelihood estimate in a streaming fashion. As long as the robot does not move, it should converge to the near-optimal likelihood field after a short period of time.

#### TODOs

* Verify that the sensor update produces consistent results on GPU and CPU
* Integrate Vulkan (mid-term)

### Motion Update

Particles are moved using odometry from TF, and we apply so-called forget factors to the likelihood estimates. While driving, the accumulated likelihood may become inaccurate, depending on the quality of your odometry estimation. To compensate, we forget old measurements so the filter can better adapt to errors. Concretely, we reduce the effective number of measurements that contribute to the current likelihood estimate by applying the forget factor (or remember factor).

#### TODOs

* Remove likelihood when sliding through walls (currently only implemented in the CPU version)
* Provide a MotionUpdater constrained to the mesh surface

### Resampling

We apply impoverishment-aware resampling using a simple method we call the `GladiatorResampler`. Each particle “fights” a randomly chosen opponent. If the opponent wins, the current particle is overwritten by the opponent and perturbed with some noise.

#### TODOs

For its simplicity, the `GladiatorResampler` performs surprisingly well and can serve as a strong baseline for more sophisticated resamplers. Possible improvements:

* Take inspiration from [MegaParticles](https://staff.aist.go.jp/k.koide/projects/icra2024_mp/) (SVGD) and use a BVH structure as an adaptive acceleration structure around the particle set for fast nearest-neighbor search. Based on some benchmarks, the BVH in [lvr2](https://github.com/uos/lvr2) may be fast enough—and it’s also GPU-capable.
* Improve strategies to reduce the number of particles more intelligently

## Software TODOs

* Test whether threading improves performance: chunk-wise operations on the particle set may allow better asynchronization
* Provide a plugin interface (mid-term)

## Contribute

We welcome contributions! If you find any of the TODOs interesting, feel free to contact me or open an issue here on GitHub.

-- [Alexander Mock](https://github.com/amock)