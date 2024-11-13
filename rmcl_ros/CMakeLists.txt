cmake_minimum_required(VERSION 3.2)
project(rmcl_ros
	VERSION 1.2.0
	)

option(BUILD_EXAMPLES "Build Examples" OFF)
option(BUILD_EXPERIMENTAL "Build Experimental Code" OFF)
option(BUILD_CONV "Build Conversion Nodes" ON)
option(BUILD_MICP_EXPERIMENTS "Build Experiments" ON)

include(GNUInstallDirs)

add_compile_options(-std=c++17)
set(CMAKE_CXX_STANDARD 17)

# DEFAULT RELEASE
if (NOT EXISTS ${CMAKE_BINARY_DIR}/CMakeCache.txt)
	if (NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
	endif()
endif()

find_package(rmcl REQUIRED)
include_directories(${rmcl_INCLUDE_DIR})

# only print warning for Rmagine version greater than RMAGINE_MAX_VERSION
set(RMAGINE_MAX_VERSION "2.3.0")

find_package(rmagine 2.2.8
    COMPONENTS
        core
    OPTIONAL_COMPONENTS
        embree
        cuda
        optix
)

if(rmagine_VERSION GREATER RMAGINE_MAX_VERSION)
    message(WARNING "Found Rmagine version: ${rmagine_VERSION} > Latest tested Rmagine version: ${RMAGINE_MAX_VERSION}. Compile at your own risk.")
else()
    message(STATUS "Rmagine Version: ${rmagine_VERSION_MAJOR}.${rmagine_VERSION_MINOR}.${rmagine_VERSION_PATCH}")
endif()

include_directories(${rmagine_INCLUDE_DIRS})

set(RMCL_ROS_DEPS
	roscpp
	geometry_msgs
	sensor_msgs
	tf2_ros
	image_transport
	visualization_msgs
	rmcl_msgs
)

find_package(catkin REQUIRED COMPONENTS
	${RMCL_ROS_DEPS}
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)

catkin_package(
    INCLUDE_DIRS 
        include
    CATKIN_DEPENDS 
        ${RMCL_ROS_DEPS}
    LIBRARIES
        rmcl_ros
)

include_directories(
    include
)

# CORE LIB ROS
add_library(rmcl_ros
    src/util/conversions.cpp
)

add_dependencies(rmcl_ros 
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
    )

target_link_libraries(rmcl_ros
    ${catkin_LIBRARIES}
    rmcl
)

install(TARGETS rmcl_ros
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

##############
#### MICP ####
##############
add_executable(micp_localization 
    src/micp_localization.cpp
    # MICP
    src/correction/MICP.cpp
    src/correction/MICPRangeSensor.cpp
)

add_dependencies(micp_localization 
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

## Specify libraries to link a library or executable target against
target_link_libraries(micp_localization
    ${catkin_LIBRARIES}
    rmcl_ros
)

if(RMCL_EMBREE)
    target_link_libraries(micp_localization
        rmcl-embree
    )
endif(RMCL_EMBREE)
    
if(RMCL_CUDA)
    target_link_libraries(micp_localization
        rmcl-cuda
    )
endif(RMCL_CUDA)

if(RMCL_OPTIX)
    target_link_libraries(micp_localization
        rmcl-optix
    )
endif(RMCL_OPTIX)

########################
### CONVERSION NODES ###
########################
if(BUILD_CONV)

    ####### PCL2 to SCAN CONVERTER
    add_executable(conv_pcl2_to_scan src/conv/pcl2_to_scan.cpp)

    add_dependencies(conv_pcl2_to_scan 
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
    )

    ## Specify libraries to link a library or executable target against
    target_link_libraries(conv_pcl2_to_scan
        ${catkin_LIBRARIES}
        rmcl_ros
    )

    ####### PCL2 to O1DN CONVERTER
    add_executable(conv_pcl2_to_o1dn src/conv/pcl2_to_o1dn.cpp)

    add_dependencies(conv_pcl2_to_o1dn 
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
    )

    ## Specify libraries to link a library or executable target against
    target_link_libraries(conv_pcl2_to_o1dn
        ${catkin_LIBRARIES}
        rmcl_ros
    )

    ####### PCL2 to DEPTH CONVERTER
    add_executable(conv_pcl2_to_depth src/conv/pcl2_to_depth.cpp)

    add_dependencies(conv_pcl2_to_depth 
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
    )

    ## Specify libraries to link a library or executable target against
    target_link_libraries(conv_pcl2_to_depth
        ${catkin_LIBRARIES}
        ${rmagine_ext_LIBRARIES}
        rmcl_ros
    )

    ####### IMAGE to DEPTH CONVERTER
    add_executable(conv_image_to_depth src/conv/image_to_depth.cpp)

    add_dependencies(conv_image_to_depth 
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
    )

    ## Specify libraries to link a library or executable target against
    target_link_libraries(conv_image_to_depth
        ${catkin_LIBRARIES}
        rmcl_ros
    )

endif(BUILD_CONV)


################
### EXAMPLES ###
################
if(BUILD_MICP_EXPERIMENTS)

if(RMCL_EMBREE)

### HILTI ###
add_executable(micp_hilti src/examples/micp_hilti.cpp)

add_dependencies(micp_hilti 
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

## Specify libraries to link a library or executable target against
target_link_libraries(micp_hilti
    ${catkin_LIBRARIES}
    rmcl-embree
)

### EVAL ###
add_executable(micp_eval src/examples/micp_eval.cpp)

add_dependencies(micp_eval 
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

## Specify libraries to link a library or executable target against
target_link_libraries(micp_eval
    ${catkin_LIBRARIES}
    rmcl-embree
)

endif(RMCL_EMBREE)

endif(BUILD_MICP_EXPERIMENTS)


if(BUILD_EXAMPLES)

if(RMCL_EMBREE)
    ###### CORRECTION CPU #######
    add_executable(ex_lidar_corrector_embree src/examples/lidar_corrector_embree.cpp)

    add_dependencies(ex_lidar_corrector_embree 
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
    )

    ## Specify libraries to link a library or executable target against
    target_link_libraries(ex_lidar_corrector_embree
        ${catkin_LIBRARIES}
        rmcl-embree
    )

    add_executable(ex_depth_corrector_embree src/examples/depth_corrector_embree.cpp)

    add_dependencies(ex_depth_corrector_embree 
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
    )

    ## Specify libraries to link a library or executable target against
    target_link_libraries(ex_depth_corrector_embree
        ${catkin_LIBRARIES}
        rmcl-embree
    )

    add_executable(ex_ondn_corrector_embree src/examples/ondn_corrector_embree.cpp)

    add_dependencies(ex_ondn_corrector_embree 
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
    )

    ## Specify libraries to link a library or executable target against
    target_link_libraries(ex_ondn_corrector_embree
        ${catkin_LIBRARIES}
        rmcl-embree
    )

    add_executable(ex_combined_corrector_embree src/examples/combined_corrector_embree.cpp)

    add_dependencies(ex_combined_corrector_embree 
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
    )

    ## Specify libraries to link a library or executable target against
    target_link_libraries(ex_combined_corrector_embree
        ${catkin_LIBRARIES}
        rmcl-embree
    )
endif(RMCL_EMBREE)

if(RMCL_OPTIX)

    add_executable(ex_lidar_corrector_optix src/examples/lidar_corrector_optix.cpp)

    add_dependencies(ex_lidar_corrector_optix 
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
    )

    ## Specify libraries to link a library or executable target against
    target_link_libraries(ex_lidar_corrector_optix
        ${catkin_LIBRARIES}
        rmcl-optix
    )

    add_executable(ex_depth_corrector_optix src/examples/depth_corrector_optix.cpp)

    add_dependencies(ex_depth_corrector_optix 
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
    )

    ## Specify libraries to link a library or executable target against
    target_link_libraries(ex_depth_corrector_optix
        ${catkin_LIBRARIES}
        rmcl-optix
    )

    add_executable(ex_ondn_corrector_optix src/examples/ondn_corrector_optix.cpp)

    add_dependencies(ex_ondn_corrector_optix 
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
    )

    ## Specify libraries to link a library or executable target against
    target_link_libraries(ex_ondn_corrector_optix
        ${catkin_LIBRARIES}
        rmcl-optix
    )

    add_executable(ex_combined_corrector_optix src/examples/combined_corrector_optix.cpp)

    add_dependencies(ex_combined_corrector_optix 
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
    )

    ## Specify libraries to link a library or executable target against
    target_link_libraries(ex_combined_corrector_optix
        ${catkin_LIBRARIES}
        rmcl-optix
    )


    # add_executable(ex_multi_lidar_corrector_optix src/examples/multi_lidar_corrector_optix.cpp)

    # add_dependencies(ex_multi_lidar_corrector_optix 
    #     ${${PROJECT_NAME}_EXPORTED_TARGETS}
    #     ${catkin_EXPORTED_TARGETS}
    # )

    # ## Specify libraries to link a library or executable target against
    # target_link_libraries(ex_multi_lidar_corrector_optix
    #     ${catkin_LIBRARIES}
    #     rmcl_optix
    # )

endif(RMCL_OPTIX)

endif(BUILD_EXAMPLES)


### TESTS ###

# add_executable(rmcl_test_cov_online src/tests/cov_online.cpp)

# ## Specify libraries to link a library or executable target against
# target_link_libraries(rmcl_test_cov_online
#     rmcl
#     rmcl_cuda
# )


# ###### CORRECTION GPU #######
# if(${rmagine_optix_FOUND})


# endif()

# #######
# # TESTS

# if(${rmagine_optix_FOUND})

# add_executable(clustering_node src/tests/clustering_node.cpp)

# add_dependencies(clustering_node 
#     ${${PROJECT_NAME}_EXPORTED_TARGETS}
#     ${catkin_EXPORTED_TARGETS}
# )

# ## Specify libraries to link a library or executable target against
# target_link_libraries(clustering_node
#     ${catkin_LIBRARIES}
#     rmcl
#     rmcl_ros
#     rmcl_optix
# )




# endif()

if(RMCL_OPTIX)
    add_executable(scan_map_segmentation_optix_node src/filter/scan_map_segmentation_optix_node.cpp)

    add_dependencies(scan_map_segmentation_optix_node 
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
    )

    ## Specify libraries to link a library or executable target against
    target_link_libraries(scan_map_segmentation_optix_node
        ${catkin_LIBRARIES}
        rmcl-optix
        rmcl_ros
    )

endif(RMCL_OPTIX)

# BENCHMARKS
if(RMCL_EMBREE)
    
    add_executable(rmcl_correction_embree_benchmark
        src/benchmarks/lidar_corrector_embree_benchmark.cpp
    )

    target_link_libraries(rmcl_correction_embree_benchmark
        rmcl-embree
    )

    add_executable(scan_map_segmentation_embree_node src/filter/scan_map_segmentation_embree_node.cpp)

    add_dependencies(scan_map_segmentation_embree_node 
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
    )

    ## Specify libraries to link a library or executable target against
    target_link_libraries(scan_map_segmentation_embree_node
        ${catkin_LIBRARIES}
        rmcl-embree
        rmcl_ros
    )

    add_executable(o1dn_map_segmentation_embree_node src/filter/o1dn_map_segmentation_embree_node.cpp)

    add_dependencies(o1dn_map_segmentation_embree_node 
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
    )

    ## Specify libraries to link a library or executable target against
    target_link_libraries(o1dn_map_segmentation_embree_node
        ${catkin_LIBRARIES}
        rmcl-embree
        rmcl_ros
    )
    
endif(RMCL_EMBREE)

# add_executable(rmcl_correction_optix_benchmark
#     src/lidar_corrector_optix_benchmark.cpp
# )

# target_link_libraries(rmcl_correction_optix_benchmark
#     rmcl_optix
# )
