cmake_minimum_required(VERSION 3.2)
project(rmcl)

# DEFAULT RELEASE
if (NOT EXISTS ${CMAKE_BINARY_DIR}/CMakeCache.txt)
  if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
  endif()
endif()

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
    src/rmcl/correction/SphereCorrectorEmbree.cpp
    src/rmcl/correction/PinholeCorrectorEmbree.cpp
)

target_link_libraries(rmcl
    ${rmagine_LIBRARY}
    ${rmagine_embree_LIBRARY}
    Eigen3::Eigen
)

add_library(rmcl_ros
    src/rmcl/util/conversions.cpp
    src/rmcl/correction/SphereCorrectorEmbreeROS.cpp
    src/rmcl/correction/PinholeCorrectorEmbreeROS.cpp
)

target_link_libraries(rmcl_ros
    ${catkin_LIBRARIES}
    rmcl
)

if(${rmagine_cuda_FOUND})
    # Build cuda related stuff
    find_package(CUDA)

    enable_language(CUDA)
    include_directories(${CUDA_INCLUDE_DIRS})

    add_library(rmcl_cuda
        # math
        src/rmcl/math/math_batched.cu
    )

    target_link_libraries(rmcl_cuda
        ${rmagine_cuda_LIBRARY}
        ${CUDA_LIBRARIES}
    )

endif()

if(${rmagine_optix_FOUND})
    # Build optix related stuff

    set(RMCL_OPTIX_PTX_DIR "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/rmcl_optix_ptx")
    set(RMCL_OPTIX_PTX_GLOB_DIR "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}/rmcl_optix_ptx")

    set(CUDA_GENERATED_OUTPUT_DIR ${RMCL_OPTIX_PTX_DIR})

    add_definitions( -DWITH_OPTIX )
    add_definitions( -DRMCL_OPTIX_PTX_DIR="${RMCL_OPTIX_PTX_DIR}" )
    add_definitions( -DRMCL_OPTIX_PTX_GLOB_DIR="${RMCL_OPTIX_PTX_GLOB_DIR}")

    message(STATUS "Writing Optix Kernels to ${RMCL_OPTIX_PTX_DIR}")

    set(OPTIX_KERNEL_FILES
        # Correction
        ## Spherical
        src/rmcl/correction/optix/SphereCorrectProgramRW.cu
        ## Pinhole
        ## O1Dn
        ## OnDn
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
        # Correction Programs
        ## Spherical
        src/rmcl/correction/optix/SphereCorrectProgramRW.cpp
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
        src/rmcl/correction/SphereCorrectorOptixROS.cpp
    )

    target_link_libraries(rmcl_optix_ros
        rmcl_optix
        rmcl_ros
    )
endif()

####### PCL2 to SCAN CONVERTER
add_executable(pcl2_to_scan src/nodes/pcl2_to_scan.cpp)

add_dependencies(pcl2_to_scan 
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

## Specify libraries to link a library or executable target against
target_link_libraries(pcl2_to_scan
    ${catkin_LIBRARIES}
    rmcl
    rmcl_ros
)

####### PCL2 to DEPTH CONVERTER
add_executable(pcl2_to_depth src/nodes/pcl2_to_depth.cpp)

add_dependencies(pcl2_to_depth 
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

## Specify libraries to link a library or executable target against
target_link_libraries(pcl2_to_depth
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

add_executable(depth_corrector_embree src/nodes/depth_corrector_embree.cpp)

add_dependencies(depth_corrector_embree 
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

## Specify libraries to link a library or executable target against
target_link_libraries(depth_corrector_embree
    ${catkin_LIBRARIES}
    ${rmagine_ext_LIBRARIES}
    rmcl
    rmcl_ros
)


###### CORRECTION GPU #######
add_executable(lidar_corrector_optix src/nodes/lidar_corrector_optix.cpp)

add_dependencies(lidar_corrector_optix 
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

## Specify libraries to link a library or executable target against
target_link_libraries(lidar_corrector_optix
    ${catkin_LIBRARIES}
    ${rmagine_ext_LIBRARIES}
    rmcl
    rmcl_ros
    rmcl_optix_ros
)

