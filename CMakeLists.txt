cmake_minimum_required(VERSION 3.2)
project(rmcl)

option(BUILD_EXAMPLES "Build Examples" OFF)
option(BUILD_EXPERIMENTAL "Build Experimental Code" OFF)
option(BUILD_CONV "Build Conversion Nodes" ON)

add_compile_options(-std=c++17)
set(CMAKE_CXX_STANDARD 17)


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

find_package(catkin REQUIRED COMPONENTS
    roscpp
    geometry_msgs
    sensor_msgs
    tf2_ros
    dynamic_reconfigure
    rmcl_msgs
    image_transport
    depth_image_proc
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)

find_package(rmagine REQUIRED)
include_directories(${rmagine_INCLUDE_DIRS})

if(rmagine_embree_FOUND)
    option(DISABLE_EMBREE "Disable Rmagine Embree backend Compilation" FALSE)
endif()

if(rmagine_cuda_FOUND)
    option(DISABLE_CUDA "Disable CUDA related compilations" FALSE)
endif()

if(rmagine_optix_FOUND)
    option(DISABLE_OPTIX "Disable Rmagine OptiX backend Compilation" FALSE)
endif()


find_package(Eigen3 REQUIRED)

### EXTERNAL PROJECTS
include(ExternalProject)

##########################################
## NanoFLANN: K-nearest neighbor search ##
##########################################
include(DownloadNanoflann)
include_directories(${nanoflann_INCLUDE_DIR})


catkin_package(
    INCLUDE_DIRS 
        include
    CATKIN_DEPENDS 
        roscpp
        geometry_msgs
        sensor_msgs
        tf2_ros
        rmcl_msgs
    DEPENDS 
        rmagine
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    include
)

# CORE LIB
add_library(rmcl
    # Math
    src/rmcl/math/math.cpp
    # Spatial
    src/rmcl/spatial/KdTree.cpp
    # Clustering
    src/rmcl/clustering/clustering.cpp
)

add_dependencies(rmcl nanoflann)

target_link_libraries(rmcl
    ${rmagine_LIBRARY}
    ${rmagine_ext_LIBRARIES}
    Eigen3::Eigen
)

# CORE LIB ROS
add_library(rmcl_ros
    src/rmcl/util/conversions.cpp
)

target_link_libraries(rmcl_ros
    ${catkin_LIBRARIES}
    rmcl
)

# EMBREE
if(${rmagine_embree_FOUND} AND NOT ${DISABLE_EMBREE})

    add_library(rmcl_embree
        # Correction
        src/rmcl/correction/SphereCorrectorEmbree.cpp
        src/rmcl/correction/PinholeCorrectorEmbree.cpp
        src/rmcl/correction/O1DnCorrectorEmbree.cpp
        src/rmcl/correction/OnDnCorrectorEmbree.cpp
    )

    target_link_libraries(rmcl_embree
        ${rmagine_LIBRARY}
        ${rmagine_embree_LIBRARY}
        ${rmagine_ext_LIBRARIES}
        rmcl
    )

    add_library(rmcl_embree_ros
        src/rmcl/correction/SphereCorrectorEmbreeROS.cpp
        src/rmcl/correction/PinholeCorrectorEmbreeROS.cpp
        src/rmcl/correction/O1DnCorrectorEmbreeROS.cpp
        src/rmcl/correction/OnDnCorrectorEmbreeROS.cpp
    )

    target_link_libraries(rmcl_embree_ros
        ${catkin_LIBRARIES}
        rmcl_embree
        rmcl_ros
    )

    add_definitions(-DRMCL_EMBREE)
    set(RMCL_EMBREE True)
endif()

# CUDA
if(${rmagine_cuda_FOUND} AND NOT ${DISABLE_CUDA})
    # Build cuda related stuff
    find_package(CUDA)

    enable_language(CUDA)
    include_directories(${CUDA_INCLUDE_DIRS})

    add_library(rmcl_cuda
        # math
        src/rmcl/math/math_batched.cu
        src/rmcl/math/math.cu
    )

    target_link_libraries(rmcl_cuda
        ${rmagine_cuda_LIBRARY}
        ${CUDA_LIBRARIES}
    )

    add_definitions(-DRMCL_CUDA)
    set(RMCL_CUDA True)
endif()

if(${rmagine_optix_FOUND} AND NOT ${DISABLE_OPTIX})
    # Build optix related stuff

    set(RMCL_OPTIX_PTX_DIR "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/rmcl_optix_ptx")
    set(RMCL_OPTIX_PTX_GLOB_DIR "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}/rmcl_optix_ptx")

    set(CUDA_GENERATED_OUTPUT_DIR ${RMCL_OPTIX_PTX_DIR})

    add_definitions( -DRMCL_OPTIX_PTX_DIR="${RMCL_OPTIX_PTX_DIR}" )
    add_definitions( -DRMCL_OPTIX_PTX_GLOB_DIR="${RMCL_OPTIX_PTX_GLOB_DIR}")

    message(STATUS "Writing Optix Kernels to ${RMCL_OPTIX_PTX_DIR}")

    set(OPTIX_KERNEL_FILES
        # Correction
        ## Spherical
        src/rmcl/correction/optix/SphereCorrectProgramRW.cu
        src/rmcl/correction/optix/SphereCorrectProgramSW.cu
        ## Pinhole
        src/rmcl/correction/optix/PinholeCorrectProgramRW.cu
        src/rmcl/correction/optix/PinholeCorrectProgramSW.cu
        ## O1Dn
        src/rmcl/correction/optix/O1DnCorrectProgramRW.cu
        src/rmcl/correction/optix/O1DnCorrectProgramSW.cu
        ## OnDn
        src/rmcl/correction/optix/OnDnCorrectProgramRW.cu
        src/rmcl/correction/optix/OnDnCorrectProgramSW.cu
    )

    cuda_compile_ptx(RMAGINE_OPTIX_PTX_FILES
        ${OPTIX_KERNEL_FILES}
    )

    add_custom_target(rmcl_optix_ptx ALL
        DEPENDS ${RMCL_OPTIX_PTX_FILES} ${OPTIX_KERNEL_FILES}
        SOURCES ${OPTIX_KERNEL_FILES}
        VERBATIM)

    message(STATUS "CMAKE_SOURCE_DIR: ${rmcl_SOURCE_DIR}")

    add_custom_command(
        TARGET rmcl_optix_ptx POST_BUILD
        COMMAND ${CMAKE_COMMAND} 
            -DRMCL_SOURCE_DIR=${rmcl_SOURCE_DIR} 
            -DRMCL_OPTIX_PTX_DIR=${RMCL_OPTIX_PTX_DIR} 
            -DOPTIX_KERNEL_FILES="${OPTIX_KERNEL_FILES}" 
            -P "${CMAKE_CURRENT_LIST_DIR}/cmake/CompileOptixKernels.cmake"
    )

    add_library(rmcl_optix
        # Correction
        src/rmcl/correction/SphereCorrectorOptix.cpp
        src/rmcl/correction/PinholeCorrectorOptix.cpp
        src/rmcl/correction/O1DnCorrectorOptix.cpp
        src/rmcl/correction/OnDnCorrectorOptix.cpp
        # Correction Programs
        src/rmcl/correction/optix/corr_modules.cpp
        src/rmcl/correction/optix/corr_program_groups.cpp
        src/rmcl/correction/optix/corr_pipelines.cpp
    )

    target_include_directories(rmcl_optix PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
    )

    target_link_libraries(rmcl_optix
        rmcl
        rmcl_cuda
        ${rmagine_optix_LIBRARY}
    )

    # ROS code
    add_library(rmcl_optix_ros
        # Sphere
        src/rmcl/correction/SphereCorrectorOptixROS.cpp
        # Pinhole
        src/rmcl/correction/PinholeCorrectorOptixROS.cpp
        # O1Dn
        src/rmcl/correction/O1DnCorrectorOptixROS.cpp
        # OnDn
        src/rmcl/correction/OnDnCorrectorOptixROS.cpp
    )

    target_link_libraries(rmcl_optix_ros
        rmcl_optix
        rmcl_ros
    )

    add_definitions(-DRMCL_OPTIX)
    set(RMCL_OPTIX True)
endif()

## Nodes 

##############
#### MICP ####
##############
add_executable(micp_localization 
    src/nodes/micp_localization.cpp
    # MICP
    src/rmcl/correction/MICP.cpp
    src/rmcl/correction/MICPRangeSensor.cpp
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
        rmcl_embree_ros
    )
endif(RMCL_EMBREE)
    
if(RMCL_CUDA)
    target_link_libraries(micp_localization
        rmcl_cuda
    )
endif(RMCL_CUDA)

if(RMCL_OPTIX)
    target_link_libraries(micp_localization
        rmcl_optix_ros
    )
endif(RMCL_OPTIX)

########################
### CONVERSION NODES ###
########################
if(BUILD_CONV)

    ####### PCL2 to SCAN CONVERTER
    add_executable(conv_pcl2_to_scan src/nodes/conv/pcl2_to_scan.cpp)

    add_dependencies(conv_pcl2_to_scan 
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
    )

    ## Specify libraries to link a library or executable target against
    target_link_libraries(conv_pcl2_to_scan
        ${catkin_LIBRARIES}
        rmcl_ros
    )

    ####### PCL2 to DEPTH CONVERTER
    add_executable(conv_pcl2_to_depth src/nodes/conv/pcl2_to_depth.cpp)

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
    add_executable(conv_image_to_depth src/nodes/conv/image_to_depth.cpp)

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
if(BUILD_EXAMPLES)

if(RMCL_EMBREE)
    ###### CORRECTION CPU #######
    add_executable(ex_lidar_corrector_embree src/nodes/examples/lidar_corrector_embree.cpp)

    add_dependencies(ex_lidar_corrector_embree 
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
    )

    ## Specify libraries to link a library or executable target against
    target_link_libraries(ex_lidar_corrector_embree
        ${catkin_LIBRARIES}
        rmcl_embree_ros
    )

    add_executable(ex_depth_corrector_embree src/nodes/examples/depth_corrector_embree.cpp)

    add_dependencies(ex_depth_corrector_embree 
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
    )

    ## Specify libraries to link a library or executable target against
    target_link_libraries(ex_depth_corrector_embree
        ${catkin_LIBRARIES}
        rmcl_embree_ros
    )

    add_executable(ex_ondn_corrector_embree src/nodes/examples/ondn_corrector_embree.cpp)

    add_dependencies(ex_ondn_corrector_embree 
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
    )

    ## Specify libraries to link a library or executable target against
    target_link_libraries(ex_ondn_corrector_embree
        ${catkin_LIBRARIES}
        rmcl_embree_ros
    )

    add_executable(ex_combined_corrector_embree src/nodes/examples/combined_corrector_embree.cpp)

    add_dependencies(ex_combined_corrector_embree 
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
    )

    ## Specify libraries to link a library or executable target against
    target_link_libraries(ex_combined_corrector_embree
        ${catkin_LIBRARIES}
        rmcl_embree_ros
    )
endif(RMCL_EMBREE)

if(RMCL_OPTIX)

    add_executable(ex_lidar_corrector_optix src/nodes/examples/lidar_corrector_optix.cpp)

    add_dependencies(ex_lidar_corrector_optix 
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
    )

    ## Specify libraries to link a library or executable target against
    target_link_libraries(ex_lidar_corrector_optix
        ${catkin_LIBRARIES}
        rmcl_optix_ros
    )

    add_executable(ex_depth_corrector_optix src/nodes/examples/depth_corrector_optix.cpp)

    add_dependencies(ex_depth_corrector_optix 
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
    )

    ## Specify libraries to link a library or executable target against
    target_link_libraries(ex_depth_corrector_optix
        ${catkin_LIBRARIES}
        rmcl_optix_ros
    )

    # add_executable(ondn_corrector_optix src/nodes/ondn_corrector_optix.cpp)

    # add_dependencies(ondn_corrector_optix 
    #     ${${PROJECT_NAME}_EXPORTED_TARGETS}
    #     ${catkin_EXPORTED_TARGETS}
    # )

    # ## Specify libraries to link a library or executable target against
    # target_link_libraries(ondn_corrector_optix
    #     ${catkin_LIBRARIES}
    #     ${rmagine_ext_LIBRARIES}
    #     rmcl
    #     rmcl_ros
    #     rmcl_optix_ros
    # )

    # add_executable(combined_corrector_optix src/nodes/combined_corrector_optix.cpp)

    # add_dependencies(combined_corrector_optix 
    #     ${${PROJECT_NAME}_EXPORTED_TARGETS}
    #     ${catkin_EXPORTED_TARGETS}
    # )

    # ## Specify libraries to link a library or executable target against
    # target_link_libraries(combined_corrector_optix
    #     ${catkin_LIBRARIES}
    #     ${rmagine_ext_LIBRARIES}
    #     rmcl
    #     rmcl_ros
    #     rmcl_optix_ros
    # )


    # add_executable(multi_lidar_corrector_optix src/nodes/multi_lidar_corrector_optix.cpp)

    # add_dependencies(multi_lidar_corrector_optix 
    #     ${${PROJECT_NAME}_EXPORTED_TARGETS}
    #     ${catkin_EXPORTED_TARGETS}
    # )

    # ## Specify libraries to link a library or executable target against
    # target_link_libraries(multi_lidar_corrector_optix
    #     ${catkin_LIBRARIES}
    #     ${rmagine_ext_LIBRARIES}
    #     rmcl
    #     rmcl_ros
    #     rmcl_optix_ros
    # )
endif(RMCL_OPTIX)

endif(BUILD_EXAMPLES)


### TESTS ###


# ###### CORRECTION GPU #######
# if(${rmagine_optix_FOUND})


# endif()

# #######
# # TESTS

# if(${rmagine_optix_FOUND})

# add_executable(clustering_node src/nodes/tests/clustering_node.cpp)

# add_dependencies(clustering_node 
#     ${${PROJECT_NAME}_EXPORTED_TARGETS}
#     ${catkin_EXPORTED_TARGETS}
# )

# ## Specify libraries to link a library or executable target against
# target_link_libraries(clustering_node
#     ${catkin_LIBRARIES}
#     rmcl
#     rmcl_ros
#     rmcl_optix_ros
# )


# add_executable(segment_node_optix src/nodes/tests/segment_node_optix.cpp)

# add_dependencies(segment_node_optix 
#     ${${PROJECT_NAME}_EXPORTED_TARGETS}
#     ${catkin_EXPORTED_TARGETS}
# )

# ## Specify libraries to link a library or executable target against
# target_link_libraries(segment_node_optix
#     ${catkin_LIBRARIES}
#     rmcl
#     rmcl_ros
#     rmcl_optix_ros
# )


# endif()




# BENCHMARKS
# add_executable(rmcl_correction_embree_benchmark
#     src/nodes/lidar_corrector_embree_benchmark.cpp
# )

# target_link_libraries(rmcl_correction_embree_benchmark
#     rmcl_embree
# )

# add_executable(rmcl_correction_optix_benchmark
#     src/nodes/lidar_corrector_optix_benchmark.cpp
# )

# target_link_libraries(rmcl_correction_optix_benchmark
#     rmcl_optix
# )
