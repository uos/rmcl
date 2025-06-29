
<div align="center" min-width=519px>
  <img src=".resources/rmcl_logo_landscape_small.png" alt="RMCL" height=150 />  
</div>


<!-- ![RMCL](.resources/rmcl_logo_landscape_small.png) -->
<div align="center">
<h4 align="center">Software Tools for Mobile Robot Localization in 3D Meshes</h4>
</div>

<div align="center">
  <a href="https://github.com/uos/rmcl">Code</a>
  <span>&nbsp;&nbsp;•&nbsp;&nbsp;</span>
  <a href="https://github.com/uos/rmcl/wiki">Documentation</a>
  <span>&nbsp;&nbsp;•&nbsp;&nbsp;</span>
  <a href="https://youtube.com/playlist?list=PL9wBuzh6ev07O2YzbjP4qbcretntl5axI">Videos</a>
  <span>&nbsp;&nbsp;•&nbsp;&nbsp;</span>
  <a href="https://github.com/uos/rmcl/issues">Issues</a>
  <span>&nbsp;&nbsp;•&nbsp;&nbsp;</span>
  <a href="https://github.com/amock/rmcl_examples">Examples</a>
  <br />
</div>

<br/>

This repository contains algorithms designed for map-based robot localization, specifically when dealing with maps composed of triangle meshes or complete scene graphs. These maps may be provided by architects who have designed the building in which the robot operates, or they can be autonomously generated by the robot through Simultaneous Localization and Mapping (SLAM) methods. It's crucial to note that map-based localization differs from SLAM; it focuses on estimating the robot's pose within a potentially large map, whether the initial pose is roughly known (tracking) or entirely unknown from the start aka kidnapped robot problem. Map-based localization is essential for precisely planning the robot's missions on a given map.

## MICP-L - Pose Tracking in Meshes

MICP-L: Mesh-based ICP for Robot Localization Using Hardware-Accelerated Ray Casting.
An approach to directly register range sensor data to a mesh in order to localize a mobile robot using hardware-accelerated ray casting correspondences (See publications).

[![Teaser](.resources/micp.gif)](http://www.youtube.com/watch?v=G-Z5K0bPFFU)

|  Hilti: 6DoF Localization  | MulRan: Large-scale scenes |
|:--:|:--:|
| <a href="http://www.youtube.com/watch?v=5pubwlbrpro" target="_blank" ><img src="https://i.ytimg.com/vi/5pubwlbrpro/maxresdefault.jpg" alt="MICP-L Hilti Video" width="100%" style="max-width: 500px" height="auto" /></a> | <a href="http://www.youtube.com/watch?v=8j6ZtYPnFzw" target="_blank" ><img src="https://i.ytimg.com/vi/8j6ZtYPnFzw/maxresdefault.jpg" alt="MICP-L MulRan Video" width="100%" style="max-width: 500px" height="auto" /></a> |

Requirements:
- At least one range sensor is equipped and running
- Triangle mesh as map
- Prior odometry estimation of the robot given as TF

IMU prior is also possible as long as it is integrated as TF-Transform, e.g. with [Madgwick Filter](http://wiki.ros.org/imu_filter_madgwick).

### Publication

Please reference the following paper when using the MICP-L method in your scientific work.

```bib
@inproceedings{mock2024micpl,
  title={{MICP-L}: Mesh-based ICP for Robot Localization Using Hardware-Accelerated Ray Casting}, 
  author={Mock, Alexander and Wiemann, Thomas and Pütz, Sebastian and Hertzberg, Joachim},
  booktitle={2024 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)}, 
  year={2024},
  pages={10664-10671},
  doi={10.1109/IROS58592.2024.10802360}
}
```

The paper is available on [IEEE Xplore](https://ieeexplore.ieee.org/document/10802360) and as preprint on [arXiv](https://arxiv.org/abs/2210.13904). The experiments are available at [https://github.com/amock/micp_experiments](https://github.com/amock/micp_experiments), but they are primarily compatible with the ROS 1 version.  
See the older branches or commits for reference.


### Running `micp_localization_node` (Theoretical Usage)

> [!NOTE]
> The following part shows a theoretical usage example meant to illustrate the general setup of MICP-L.  
> For actual working examples and detailed instructions, please refer to:
> [https://github.com/amock/rmcl_examples](https://github.com/amock/rmcl_examples)


The `micp_localization_node` starts the process of localizing the robot within a triangle mesh using MICP, based on a given pose estimate.  It is typically launched via a **launch file**:

```xml
<launch>
  <node pkg="rmcl_ros" exec="micp_localization_node" name="rmcl_micpl" output="screen">
    <param name="map_file" value="/path/to/mesh/map.dae" />
    <param from="/path/to/config/file.yaml" />
  </node>
</launch>
```

<details>
<summary>Once the launch file is started, the output in Terminal should look as follows:</summary>

```console
[micp_localization_node-2] -------------------------
[micp_localization_node-2]        --- MAP ---       
[micp_localization_node-2] -------------------------
[micp_localization_node-2] - file: /home/amock/rmcl_ws/install/rmcl_examples_maps/share/rmcl_examples_maps/maps/tray.dae
[micp_localization_node-2] - meshes: 1
[micp_localization_node-2] Cube-mesh
[micp_localization_node-2]   - vertices, faces: 30, 10
[micp_localization_node-2] For more infos enter in terminal: 
[micp_localization_node-2] $ rmagine_map_info /home/amock/rmcl_ws/install/rmcl_examples_maps/share/rmcl_examples_maps/maps/tray.dae
[micp_localization_node-2] 
[micp_localization_node-2] --------------------------
[micp_localization_node-2]      --- BACKENDS ---     
[micp_localization_node-2] --------------------------
[micp_localization_node-2] Available combining units:
[micp_localization_node-2] - CPU
[micp_localization_node-2] Available raytracing backends:
[micp_localization_node-2] - Embree (CPU)
[micp_localization_node-2] 
[micp_localization_node-2] -------------------------
[micp_localization_node-2]      --- FRAMES ---      
[micp_localization_node-2] -------------------------
[micp_localization_node-2] - base:	base_footprint
[micp_localization_node-2] - odom:	odom
[micp_localization_node-2] - map:	map
[micp_localization_node-2] Estimating: base_footprint -> map
[micp_localization_node-2] Providing:  odom -> map
[micp_localization_node-2] 
[micp_localization_node-2] -------------------------
[micp_localization_node-2]      --- SENSORS ---     
[micp_localization_node-2] -------------------------
[micp_localization_node-2] - lidar3d
[micp_localization_node-2]   - data:	topic
[micp_localization_node-2]     - topic:	/rmcl_inputs/lidar3d
[micp_localization_node-2]     - frame:	velodyne
[micp_localization_node-2]   - model:	o1dn
[micp_localization_node-2]   - correspondences: 
[micp_localization_node-2]      - backend: embree
[micp_localization_node-2]      - type:    RC
[micp_localization_node-2]      - metric:  P2L
[micp_localization_node-2] MICP load params - done. Valid Sensors: 1
[micp_localization_node-2] [INFO] [1747438141.203392843] [rmcl_micpl]: Waiting for 'odom' frame to become available ...
[micp_localization_node-2] Waiting for pose...
```
</details>


After the node has been started, an initial pose estimate must be provided, eg, using the "2D Pose Estimate" tool in RViz, which publishes to the `/initialpose` topic.
Make sure the fixed frame in RViz is set to match the map coordinate system.

> Note: RMCL does not provide tools to visualize triangle mesh maps in RViz.
> To view mesh maps, consider using the rviz_mesh_tools_plugins from the
[mesh_tools](https://github.com/naturerobots/mesh_tools) repository.

--> For an actual quick start, go to: [https://github.com/amock/rmcl_examples](https://github.com/amock/rmcl_examples)


# RMCL - Project

## Installation

Dependencies:
- ROS 2 (check compatible branches)
- Download and put [Rmagine](https://github.com/uos/rmagine) (v >= 2.3.0) into your ROS workspace.
  - Recommended: Install OptiX backend if NVIDIA GPU is available.
- Optional for functionality, but required for visualizations: [mesh_tools](https://github.com/naturerobots/mesh_tools).

Clone this repository into your ROS workspace and build it.

```console
colcon build
```

### Branch Compatibility

|  RMCL Branch    |  Supported ROS 2 versions    |
|:----|:----|
|  main   |  humble, jazzy |

## Mesh Navigation

To navigate a robot automatically and safely through uneven terrain, the combination RMCL + Mesh Navigation Stack is very suitable: [https://github.com/naturerobots/mesh_navigation](https://github.com/naturerobots/mesh_navigation). As we presented on [ROSCon 2023](https://vimeo.com/879000775):

<a href="https://vimeo.com/879000775" target="_blank" ><img src=".resources/ROSCon2023.png" alt="MICP-L ROSCon 2023 Video" width="300px" /></a>

## Roadmap

This package will be expanded by more functionalities to localize a robot in mesh maps.
The planned Roadmap is as follows:

- [x] MICP-L (Pose Tracking)
- [ ] RMCL (Global Localization)

## News

### 2025-05-17: ROS 2-ify MICP-L - v2.2.0

After conducting real-world tests, we refactored the **MICP-L** node to better integrate it into the **ROS 2** ecosystem and added several new features:
- Limited the possible inputs to *only* `rmcl_msgs`. Instead, we provide nodes and instructions to convert commonly used range sensor messages into `rmcl_msgs`.
- MICP-L can now be launched as a composable node.
- Separated correspondence search from optimization without losing much efficiency. This allowed us to add classic closest-point correspondences (CP), in addition to ray-casting correspondences (RC) (only available for embree backend).
- Improved time synchronization between combinations of sensors and odometry.
- Added many new examples and small demos for a quick start: [https://github.com/amock/rmcl_examples](https://github.com/amock/rmcl_examples)

> For the old version, download v2.1.0

### 2024-11-25: Restructuring - ROS 1 + ROS 2

We had to do minor structural changes to the repository in order to better integrate new features into RMCL. This repository is now devided into
- "rmcl" which is a ROS-agnostic library that can be compiled and installed as regular CMake project,
- "rmcl_ros" which contains all the nodes,
- "rmcl_msgs" which are the message moved from to this repository. The original msgs repository is not required anymore.

Using the latest rmcl version might break your launch files as the nodes are now located in "rmcl_ros" package. However, it's rather simple to fix that.
The new versions of RMCL are v2.1.0 for ROS 2 and v1.3.0 for ROS 1.

### 2024-02-11: ROS2 release - v2.0.0

The main branch is humble now! Since it is not backwards compatible we decided to increase the version of RMCL to 2.0.0. The noetic version will still exist with the "noetic" branch. The "noetic" branch will be maintained until the end of 2024.

### 2024-01-05: ROS2 - humble

The first ROS2 port has been released! If you are using ROS2, check out the `humble` branch of this and all linked repositories. After the new branch has been tested well enough, I will make it the main branch. The current version will persist in the `noetic` branch.
