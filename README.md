[[Code](https://github.com/uos/rmcl)] [[Wiki](https://github.com/uos/rmcl/wiki)]

# RMCL

Software Tools for Mobile Robot Localization in 3D Meshes.


## Installation

Dependencies:
- Download and install [Rmagine](https://github.com/uos/rmagine)
  - Recommended: Install OptiX backend if NVIDIA GPU is available
- ROS (tested with ROS-noetic)
- Clone [rmcl_msgs](https://github.com/uos/rmcl_msgs) to your workspace

Clone this repository into your ROS workspace and build it.

# MICP-L

Mesh-ICP Localization. Direct Range sensor to mesh registration for localization using simulative projective correspondences (See publications).

Requirements:
- At least one Range Sensor equipped and running
- Triangle Mesh as map
- Odometry estimation of the robot given as TF-Frame

![Teaser](dat/micp.gif)

Full Video is available on Youtube soon.

## Publication

- Title: "MICP-L: Fast parallel simulative Range Sensor to Mesh Registration for Robot Localization"
- State: Submitted to ICRA 2023
- Preprint: https://arxiv.org/abs/2210.13904

## Usage

The `micp_localization` Node starts the process of localizing the robot in a mesh using MICP and a given pose estimate. 
It is usually started through a Launch-File since it requires a large set of parameters.

### Launch

Starting the following Launch-File

```xml
<launch>

<arg name="map" default="$(find uos_gazebo_worlds)/Media/models/avz_neu.dae" />
<arg name="config" default="$(find rmcl)/config/examples/micp_velodyne_cpu.yaml" />

<node pkg="rmcl" type="micp_localization" name="micp_localization" output="screen">
    <param name="map_file" type="string" value="$(arg map)" />
    <rosparam command="load" file="$(arg config)" />
    <remap from="pose_wc" to="/initialpose" />
</node>

</launch>
```

runs the MICP localization. After that a pose has to be given, e.g. by the RViz "2D Pose Estimate" Tool that publishes the results on the `/initialpose` topic.
Doing that, make sure to the set the Fixed Frame to the map coordinate system.
RMCL itself doesn't provide any tools to visualize the maps (triangle meshes).
If you want to see the map in RViz, use for example the `rviz_mesh_plugin` of the [mesh_tools](https://github.com/uos/mesh_tools).

Once the launch file is started, the output in Terminal should look as follows:

```console
Combining Unit: CPU
MICP initiailized

-------------------------
    --- BACKENDS ---    
-------------------------
Available computing units:
- CPU
- GPU
Available raytracing backends:
- Embree (CPU)
- Optix (GPU)
MICP load params

-------------------------
     --- FRAMES ---      
-------------------------
- base:			base_footprint
- odom:			odom
  - base -> odom:	yes
- map:			map
Estimating: base_footprint -> map
Providing: odom -> map

-------------------------
     --- SENSORS ---     
-------------------------
- velodyne
  - data:		Topic
    - topic:		/velodyne_points
    - msg:		sensor_msgs/PointCloud2
    - data:		yes
    - frame:		velodyne
  - model:		Params
  - type:		spherical - loaded
  - micp:
    - backend:		embree
MICP load params - done. Valid Sensors: 1
Waiting for pose guess...
```

At startup, MICP does a few sanity checks for the input parameters.
Every possible mistake in configuration can then be inferred by this output.
For example, once there is no data available on the given `PointCloud2`-Topic it will print `data: no` instead.


### Params

The following sections describe example configuration files.
More example files for configuration are placed in the `config/examples`.

### Params - 3D LiDaR only - CPU

MICP Localization using a 3D LiDaR and doing the MICP steps completely on the CPU.
Here the 3D LiDaR is a Velodyne VLP-16 with 16 scan lines.
The horizontal number of points are reduced to 440 and might be adjusted for your own Velodyne.


File: `config/examples/micp_velodyne_cpu.yaml`

```yaml
# required
base_frame: base_footprint
map_frame: map
odom_frame: odom

# rate of broadcasting tf transformations
tf_rate: 50
invert_tf: False

micp:
  # merging on gpu or cpu
  combining_unit: cpu
  # maximum number of correction steps per second
  # lower this to decrease the correction speed but save energy 
  corr_rate_max: 1000
  print_corr_rate: False

  # adjust max distance dependend of the state of localization
  adaptive_max_dist: True # enable adaptive max dist

  # offset added to inital pose guess
  trans: [0.0, 0.0, 0.0]
  rot: [0.0, 0.0, 0.0] # euler angles (3) or quaternion (4)  

# describe your sensor setup here
sensors: # list of range sensors - at least one is required
  velodyne:
    topic: velodyne_points
    type: spherical
    model:
      range_min: 0.5
      range_max: 130.0
      phi_min: -0.261799067259
      phi_inc: 0.03490658503988659
      phi_N: 16
      theta_min: -3.14159011841
      theta_inc: 0.01431249500496489 # (2*pi)/439 instead of (2*pi)/440 
      theta_N: 440
    micp:
      max_dist: 1.0
      adaptive_max_dist_min: 0.15
      backend: embree
```

### Params - 2D LiDaR + Wheels - GPU

MICP also supports to localize a robot only equipped with a 2D LiDaR in a 3D map.
To correct the third dimension the wheels can be used to pull the robot towards the map's ground plane. 
Thus, you should only run it on a robot that always drives on ground and e.g. cannot fly.
In this example, all MICP steps are computed on GPU.
The robot has four wheels of which the highest points are located relative to `base_footprint` as listed in `origs`.
By setting a virtual scanner to this wheel positions scanning downwards with a range equals the wheel diameter it is possible to pull the robot to the mesh.


File: `config/examples/micp_sick_gpu.yaml`

```yaml
# required
base_frame: base_footprint
map_frame: map
odom_frame: odom

# rate of broadcasting tf transformations
tf_rate: 50

micp:
  # merging on gpu or cpu
  combining_unit: gpu
  # maximum number of correction steps per second
  # lower this to decrease the correction speed but save energy 
  corr_rate_max: 1000
  print_corr_rate: False

  # adjust max distance dependend of the state of localization
  adaptive_max_dist: True # enable adaptive max dist

  # offset added to initial pose guess
  trans: [0.0, 0.0, 0.0]
  rot: [0.0, 0.0, 0.0] # euler angles (3) or quaternion (4)  

# describe your sensor setup here
sensors: # list of range sensors - at least one is required
  sick:
    topic: scan
    micp:
      weight: 1
      backend: optix
  wheels: # pull robot to the mesh
    ranges: [0.2, 0.2, 0.2, 0.2]
    type: ondn
    frame: base_footprint
    model:
      width: 4
      height: 1
      range_min: 0.0
      range_max: 10.0
      origs: [[ 0.2,  0.15,  0.2], # front left 
              [ 0.2, -0.15,  0.2], # front right
              [-0.2,  0.15,  0.2], # rear left
              [-0.2, -0.15,  0.2]] # rear right
      dirs:  [[ 0.0,  0.0, -1.0],
              [ 0.0,  0.0, -1.0],
              [ 0.0,  0.0, -1.0],
              [ 0.0,  0.0, -1.0]]
    micp:
      max_dist: 1.0
      adaptive_max_dist_min: 0.2
      weight: 1
      backend: optix
```

## Examples

To learn how to use RMCL ROS nodes in your project, visit https://github.com/aock/rmcl_example.

To learn how to use RMCL library in your Node: `src/nodes/examples`.

# Roadmap

This package will be expanded by more functionalities to localize a robot in mesh maps.
The planned Roadmap is as follows:

- [x] MICP-L
- [ ] RMCL



