cmake_minimum_required(VERSION 3.2)
project(rmcl)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    geometry_msgs
    sensor_msgs
    tf2_ros
    dynamic_reconfigure
    mesh_msgs
    rmcl_msgs
    rosmath
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)

find_package(rmagine REQUIRED)
include_directories(${rmagine_INCLUDE_DIRS})

find_package(Eigen3 REQUIRED)

catkin_package(
    INCLUDE_DIRS 
        # include
    CATKIN_DEPENDS 
        roscpp
        geometry_msgs
        sensor_msgs
        tf2_ros
        rmcl_msgs
        rosmath
    DEPENDS 
        rmagine
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    include
)

add_library(rmcl
    # Correction
    src/rmcl/correction/LiDARCorrectorEmbree.cpp
)

target_link_libraries(rmcl
    ${rmagine_LIBRARY}
    ${rmagine_embree_LIBRARY}
    Eigen3::Eigen
)

add_library(rmcl_ros
    src/rmcl/util/conversions.cpp
    src/rmcl/correction/LiDARCorrectorEmbreeROS.cpp
)

target_link_libraries(rmcl_ros
    ${catkin_LIBRARIES}
    rmcl
)

####### PCL CONVERTER
add_executable(pcl_converter src/nodes/pcl_converter.cpp)

add_dependencies(pcl_converter 
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

## Specify libraries to link a library or executable target against
target_link_libraries(pcl_converter
    ${catkin_LIBRARIES}
    rmcl
    rmcl_ros
)

###### CORRECTION CPU #######
add_executable(lidar_corrector_embree src/nodes/lidar_corrector_embree.cpp)

add_dependencies(lidar_corrector_embree 
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

## Specify libraries to link a library or executable target against
target_link_libraries(lidar_corrector_embree
    ${catkin_LIBRARIES}
    ${rmagine_ext_LIBRARIES}
    rmcl
    rmcl_ros
)


