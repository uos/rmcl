# Raycasting Monte Carlo Localization (RMCL)

> Note: RMCL is under development and heavy testing. 
> Nevertheless we decided to pre-release the `v0` version in an unfinished state, because we think it can help other to see 
> whats possible. Also we hope you can test this and perhaps help us in improving / finishing the last things.

This node provides a practical, real-time implementation of Monte Carlo Localization (MCL) for global robot localization, accelerated by high-performance ray tracing over triangle meshes and geometric scene graphs. MCL has a decades-long track record; our focus is making it easy to deploy and tune on real robots. The pipeline scales across diverse hardware with parameters to meet tight compute and memory budgets (including for our smallest robots). The documentation begins with hands-on usage and configuration of `rmcl_localization_node`, followed by a concise overview of the underlying concepts and design choices.

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
- rmcl/initial_pose_guess





## Theory

TODO: Add an overview here



### Motion Update

### Sensor Update

### Resampling





