cmake_minimum_required(VERSION 3.18)
project(rmcl_ros
	VERSION 2.1.0)

option(BUILD_EXAMPLES "Build Examples" OFF)
option(BUILD_EXPERIMENTAL "Build Experimental Code" OFF)
option(BUILD_CONV "Build Conversion Nodes" ON)
option(BUILD_MICP_EXPERIMENTS "Build Experiments" ON)


include(GNUInstallDirs)

add_compile_options(-std=c++17)
# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

message(STATUS "CMake Version: ${CMAKE_VERSION}")
if(CMAKE_VERSION VERSION_GREATER_EQUAL 3.9)
    message(STATUS ">= 3.9 - Enabling Link Time Optimization")
    set(CMAKE_INTERPROCEDURAL_OPTIMIZATION TRUE)
endif()

# DEFAULT RELEASE
if (NOT EXISTS ${CMAKE_BINARY_DIR}/CMakeCache.txt)
  if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
  endif()
endif()

set(CMAKE_MODULE_PATH
  ${rmcl_SOURCE_DIR}/cmake
  ${CMAKE_MODULE_PATH}
)

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rmcl_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(OpenMP REQUIRED)


find_package(rmagine 2.2.12
    COMPONENTS
        core
    OPTIONAL_COMPONENTS
        embree
        cuda
        optix
)

find_package(rmcl 2.1 REQUIRED)
include_directories(${rmcl_INCLUDE_DIR})

# TODO: we can do this better. maybe but definitions to each target?
if(TARGET rmagine::embree)
    add_definitions(-DRMCL_EMBREE)
    set(RMCL_EMBREE TRUE)
endif(TARGET rmagine::embree)

if(TARGET rmcl-cuda)
    add_definitions(-DRMCL_CUDA)
    set(RMCL_CUDA True)
endif(TARGET rmcl-cuda)

if(TARGET rmagine::optix)
    add_definitions(-DRMCL_OPTIX)
    set(RMCL_OPTIX True)
endif(TARGET rmagine::optix)

# find_package(Eigen3 REQUIRED)

include_directories(
    include
)

# CORE LIB ROS
add_library(rmcl_ros SHARED
    src/util/conversions.cpp
    src/util/scan_operations.cpp
    src/util/ros_helper.cpp
    # micp localization
    src/correction/MICPSensor.cpp
    src/correction/sensors/MICPO1DnSensorCPU.cpp
    src/correction/sensors/MICPSphericalSensorCPU.cpp
    src/correction/correspondences/CorrespondencesCPU.cpp
)

target_include_directories(rmcl_ros PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")

target_link_libraries(rmcl_ros
    rmcl
)

ament_target_dependencies(rmcl_ros
    rclcpp
    sensor_msgs
    rmcl_msgs
    geometry_msgs
    tf2_ros
)

install(TARGETS rmcl_ros
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

if(RMCL_EMBREE)
    add_library(rmcl_ros_embree SHARED
        src/correction/correspondences/RCCEmbree.cpp
        src/correction/correspondences/CPCEmbree.cpp
    )
    target_include_directories(rmcl_ros_embree PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include>")
    target_link_libraries(rmcl_ros_embree
        rmcl_ros
        rmagine::embree
        OpenMP::OpenMP_CXX
    )

    ament_target_dependencies(rmcl_ros_embree
        rclcpp
        sensor_msgs
        rmcl_msgs
        geometry_msgs
    )

    install(TARGETS rmcl_ros_embree
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
    )
endif(RMCL_EMBREE)

if(RMCL_CUDA)
    add_library(rmcl_ros_cuda SHARED
        src/correction/correspondences/CorrespondencesCUDA.cpp
        src/correction/sensors/MICPO1DnSensorCUDA.cpp
    )
    target_include_directories(rmcl_ros_cuda PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include>")
    target_link_libraries(rmcl_ros_cuda
        rmcl_ros
        rmcl-cuda
        rmagine::cuda
    )

    ament_target_dependencies(rmcl_ros_cuda
        rclcpp
        sensor_msgs
        rmcl_msgs
        geometry_msgs
    )

    install(TARGETS rmcl_ros_cuda
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
    )
endif(RMCL_CUDA)

if(RMCL_OPTIX)
    add_library(rmcl_ros_optix SHARED
        src/correction/correspondences/RCCOptix.cpp
    )
    target_include_directories(rmcl_ros_optix PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include>")
    target_link_libraries(rmcl_ros_optix
        rmcl_ros
        rmagine::optix
    )

    ament_target_dependencies(rmcl_ros_optix
        rclcpp
        sensor_msgs
        rmcl_msgs
        geometry_msgs
    )

    install(TARGETS rmcl_ros_optix
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
    )
endif(RMCL_OPTIX)

## Nodes

#####################
#### MICP-L Node ####
#####################
add_library(micp_localization SHARED
    src/nodes/micp_localization.cpp
)

target_include_directories(micp_localization
  PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

target_link_libraries(micp_localization
    rmcl_ros
    OpenMP::OpenMP_CXX
)

if(RMCL_EMBREE)
    target_link_libraries(micp_localization
        rmcl_ros_embree
    )
endif(RMCL_EMBREE)
    
if(RMCL_CUDA)
    target_link_libraries(micp_localization
        rmcl_ros_cuda
    )
endif(RMCL_CUDA)

if(RMCL_OPTIX)
    target_link_libraries(micp_localization
        rmcl_ros_optix
    )
endif(RMCL_OPTIX)

ament_target_dependencies(micp_localization
    rclcpp
    rclcpp_components
    geometry_msgs
    sensor_msgs
    tf2_ros
    rmcl_msgs
    image_transport
    visualization_msgs
)

rclcpp_components_register_node(micp_localization
    PLUGIN "rmcl::MICPLocalizationNode" 
    EXECUTABLE micp_localization_node)

install(TARGETS 
    micp_localization
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

# TODO: PORT TO ROS2!
# ########################
# ### CONVERSION NODES ###
# ########################
if(BUILD_CONV)

    ####### PC2 to SCAN CONVERTER
    add_library(conv_pc2_to_scan SHARED 
        src/nodes/conversion/pc2_to_scan.cpp)

    ## Specify libraries to link a library or executable target against
    target_link_libraries(conv_pc2_to_scan
        rmcl_ros
    )

    ament_target_dependencies(conv_pc2_to_scan
        rclcpp
        rclcpp_components
        geometry_msgs
        sensor_msgs
        tf2
        tf2_ros
        rmcl_msgs
        image_transport
        visualization_msgs
    )

    rclcpp_components_register_node(conv_pc2_to_scan 
        PLUGIN "rmcl::Pc2ToScanNode" 
        EXECUTABLE conv_pc2_to_scan_node)

    install(TARGETS 
        conv_pc2_to_scan
      ARCHIVE DESTINATION lib
      LIBRARY DESTINATION lib
      RUNTIME DESTINATION bin
    )


    ####### PC2 to O1DN CONVERTER
    add_library(conv_pc2_to_o1dn SHARED 
        src/nodes/conversion/pc2_to_o1dn.cpp)

    ## Specify libraries to link a library or executable target against
    target_link_libraries(conv_pc2_to_o1dn
        rmcl_ros
    )

    ament_target_dependencies(conv_pc2_to_o1dn
        rclcpp
        rclcpp_components
        geometry_msgs
        sensor_msgs
        tf2
        tf2_ros
        rmcl_msgs
        image_transport
        visualization_msgs
    )

    rclcpp_components_register_node(conv_pc2_to_o1dn 
        PLUGIN "rmcl::Pc2ToO1DnNode" 
        EXECUTABLE conv_pc2_to_o1dn_node)

    install(TARGETS 
        conv_pc2_to_o1dn
      ARCHIVE DESTINATION lib
      LIBRARY DESTINATION lib
      RUNTIME DESTINATION bin      
    )

    ####### SCAN to SCAN CONVERTER
    add_library(conv_scan_to_scan SHARED 
        src/nodes/conversion/scan_to_scan.cpp)

    ## Specify libraries to link a library or executable target against
    target_link_libraries(conv_scan_to_scan
        rmcl_ros
    )

    ament_target_dependencies(conv_scan_to_scan
        rclcpp
        rclcpp_components
        geometry_msgs
        sensor_msgs
        tf2
        tf2_ros
        rmcl_msgs
        image_transport
        visualization_msgs
    )

    rclcpp_components_register_node(conv_scan_to_scan 
        PLUGIN "rmcl::ScanToScanNode" 
        EXECUTABLE conv_scan_to_scan_node)

    install(TARGETS 
        conv_scan_to_scan
      ARCHIVE DESTINATION lib
      LIBRARY DESTINATION lib
      RUNTIME DESTINATION bin
    )

    
    ####### PC2 to DEPTH CONVERTER
    # add_executable(conv_pc2_to_depth src/nodes/conv/pc2_to_depth.cpp)

    ## Specify libraries to link a library or executable target against
    # target_link_libraries(conv_pc2_to_depth
    #     rmagine::core
    #     rmcl_ros
    # )

    # ament_target_dependencies(conv_pc2_to_depth
    #     rclcpp
    #     geometry_msgs
    #     sensor_msgs
    #     tf2_ros
    #     rmcl_msgs
    #     image_transport
    #     visualization_msgs
    # )

    # install(TARGETS 
    #     conv_pc2_to_depth
    #     DESTINATION lib/${PROJECT_NAME})
        
    ####### IMAGE to DEPTH CONVERTER
    # add_executable(conv_image_to_depth src/nodes/conv/image_to_depth.cpp)

    # add_dependencies(conv_image_to_depth 
    #     ${${PROJECT_NAME}_EXPORTED_TARGETS}
    #     ${catkin_EXPORTED_TARGETS}
    # )

    # ## Specify libraries to link a library or executable target against
    # target_link_libraries(conv_image_to_depth
    #     ${catkin_LIBRARIES}
    #     rmcl_ros
    # )

endif(BUILD_CONV)

# ################
# ### EXAMPLES ###
# ################
if(BUILD_MICP_EXPERIMENTS)

if(TARGET rmcl-embree)

### EVAL ###
add_executable(micp_eval src/nodes/micp_eval.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(micp_eval
    rmcl-embree
    rmcl_ros
)

ament_target_dependencies(micp_eval
    rclcpp
    geometry_msgs
    sensor_msgs
    tf2_ros
    rmcl_msgs
    image_transport
    visualization_msgs
)

install(TARGETS 
    micp_eval
    DESTINATION lib/${PROJECT_NAME})

endif(TARGET rmcl-embree)

endif(BUILD_MICP_EXPERIMENTS)

# TODO: add more segmentation nodes
if(TARGET rmcl-embree)

add_library(o1dn_map_segmentation_embree SHARED
    src/nodes/filter/o1dn_map_segmentation_embree_node.cpp)

target_link_libraries(o1dn_map_segmentation_embree
    rmcl_ros
    rmcl-embree
)

ament_target_dependencies(o1dn_map_segmentation_embree
    rclcpp
    rclcpp_components
    geometry_msgs
    sensor_msgs
    tf2_ros
    rmcl_msgs
)

rclcpp_components_register_node(o1dn_map_segmentation_embree 
    PLUGIN "rmcl::O1DnMapSegmentationEmbreeNode" 
    EXECUTABLE o1dn_map_segmentation_embree_node)

install(TARGETS 
    o1dn_map_segmentation_embree
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin      
)

add_library(scan_map_segmentation_embree SHARED
    src/nodes/filter/o1dn_map_segmentation_embree_node.cpp)

target_link_libraries(scan_map_segmentation_embree
    rmcl_ros
    rmcl-embree
)

ament_target_dependencies(scan_map_segmentation_embree
    rclcpp
    rclcpp_components
    geometry_msgs
    sensor_msgs
    tf2_ros
    rmcl_msgs
)

rclcpp_components_register_node(scan_map_segmentation_embree 
    PLUGIN "rmcl::ScanMapSegmentationEmbreeNode" 
    EXECUTABLE scan_map_segmentation_embree_node)

install(TARGETS 
    o1dn_map_segmentation_embree
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin      
)

endif(TARGET rmcl-embree)


install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(
  include
)

ament_export_libraries(
  rmcl_ros
)

ament_export_dependencies(rclcpp
  rclcpp_components
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
  rmcl_msgs
  image_transport
  visualization_msgs)

ament_package()
