# Mesh ICP Localization (MICP-L)



## Running `micp_localization_node` (Theoretical Usage)

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



## Publication

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