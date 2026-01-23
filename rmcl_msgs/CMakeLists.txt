cmake_minimum_required(VERSION 3.8)
project(rmcl_msgs VERSION 2.4.0)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/PolarCoord.msg"
    "msg/RangeData.msg"
    "msg/RangeDataStamped.msg"
    # Spherical
    "msg/ScanInfo.msg"
    "msg/ScanInfoStamped.msg"
    "msg/Scan.msg"
    "msg/ScanStamped.msg"
    # Pinhole
    "msg/DepthInfo.msg"
    "msg/DepthInfoStamped.msg"
    "msg/Depth.msg"
    "msg/DepthStamped.msg"
    # O1Dn
    "msg/O1DnInfo.msg"
    "msg/O1DnInfoStamped.msg"
    "msg/O1Dn.msg"
    "msg/O1DnStamped.msg"
    # OnDn
    "msg/OnDnInfo.msg"
    "msg/OnDnInfoStamped.msg"
    "msg/OnDn.msg"
    "msg/OnDnStamped.msg"
    # MICP
    "msg/MICPSensorStats.msg"
    "msg/MICPStats.msg"
    # RMCL
    "srv/SetInitialPose.srv"
    "msg/LikelihoodStats.msg"
    "msg/ParticleStats.msg"
  DEPENDENCIES
    std_msgs
    geometry_msgs
    sensor_msgs
  ADD_LINTER_TESTS
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
