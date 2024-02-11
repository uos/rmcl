^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package RMCL
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.0 (2024-02-11)
------------------
First ROS2 release. Version humble
* Noetic version will be maintained until end of 2024

1.1.3 (2023-12-04)
------------------
* added findRCC functions to all embree simulators
* added rmagine dependency to package.xml since rmagine can now compiled with a ROS workspace

1.1.3 (2023-10-08)
------------------
Patches:
* Use new rmagine cmake components. Minumum version 2.2.1 required.
* Bug fix for CPU-only machines

1.1.2 (2023-05-03)
------------------
* Optimization: Integrated Rmagine versioning and warnings for potential Rmagine version mismatch. Disabled examples

1.1.1 (2022-10-28)
------------------
* Optimization: Inaccurate Covariances on GPU

1.1.0 (2022-10-26)
------------------
Interface Changes:
* every function has now the order of arguments: dataset first, model second
Bug Fixes:
* correspondences were computed incorrectly
* tf rate influenced the correction rate
New Functionalities:
* added findSPC function for computing the correspondences seperately
* added one-pass and two-pass means and covariance computation
Application Changes:
* In MICP Node RViz visualizations can be enabled
* In MICP Node Correction can be stopped to better visualize correspondences

1.0.0 (2022-10-18)
------------------
First release, contributors: Alexander Mock
* Feature: Mesh ICP Localization (MICP-L)
* Registering any Range sensor to a Triangle Mesh
* Use CPU only or Hardware Acceleration on NVIDIA GPUs with OptiX