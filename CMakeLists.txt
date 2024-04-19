cmake_minimum_required(VERSION 3.18)
project(rmcl
	VERSION 2.0.1)

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

# only print warning for Rmagine version greater than RMAGINE_MAX_VERSION
set(RMAGINE_MAX_VERSION "2.4.0")

find_package(rmagine 2.2.3
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

if(TARGET rmagine::embree)
    option(DISABLE_EMBREE "Disable Rmagine Embree backend Compilation" FALSE)
endif()

if(TARGET rmagine::cuda)
    option(DISABLE_CUDA "Disable CUDA related compilations" FALSE)
endif()

if(TARGET rmagine::optix)
    option(DISABLE_OPTIX "Disable Rmagine OptiX backend Compilation" FALSE)
endif()

find_package(Eigen3 REQUIRED)

### EXTERNAL PROJECTS
include(ExternalProject)

##########################################
## NanoFLANN: K-nearest neighbor search ##
##########################################
# TODO: fix nanoflann. 
# - We dont need it right now -> remove at first
# include(DownloadNanoflann)
# include_directories(${nanoflann_INCLUDE_DIR})

set(RMCL_LIBRARIES rmcl rmcl_ros)

if(TARGET rmagine::embree AND NOT ${DISABLE_EMBREE})
    list(APPEND RMCL_LIBRARIES rmcl_embree)
endif()

if(TARGET rmagine::cuda AND NOT ${DISABLE_CUDA})
    list(APPEND RMCL_LIBRARIES rmcl_cuda)
endif()

if(TARGET rmagine::optix AND NOT ${DISABLE_OPTIX})
    list(APPEND RMCL_LIBRARIES rmcl_optix)
endif()

include_directories(
    include
)

# CORE LIB
add_library(rmcl
    # Math
    src/rmcl/math/math.cpp
    src/rmcl/math/math_batched.cpp
    # # Spatial
    # src/rmcl/spatial/KdTree.cpp # TODO: fix nanoflann
    # # Clustering
    # src/rmcl/clustering/clustering.cpp # TODO: fix nanoflann
)

target_compile_definitions(rmcl PRIVATE "RMCL_BUILDING_LIBRARY")#

target_include_directories(rmcl PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")

target_link_libraries(rmcl
    rmagine::core
    Eigen3::Eigen
)

install(TARGETS rmcl
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

list(APPEND RMCL_LIBS rmcl)

# CORE LIB ROS
add_library(rmcl_ros
    src/rmcl/util/conversions.cpp
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
)

install(TARGETS rmcl_ros
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

list(APPEND RMCL_LIBS rmcl_ros)

# EMBREE
if(TARGET rmagine::embree)

    add_library(rmcl_embree
        # Correction
        src/rmcl/correction/SphereCorrectorEmbree.cpp
        src/rmcl/correction/PinholeCorrectorEmbree.cpp
        src/rmcl/correction/O1DnCorrectorEmbree.cpp
        src/rmcl/correction/OnDnCorrectorEmbree.cpp
    )

    target_link_libraries(rmcl_embree
        rmagine::core
        rmagine::embree
        ${rmagine_ext_LIBRARIES}
        rmcl
    )

    install(TARGETS rmcl_embree
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
    )

    list(APPEND RMCL_LIBS rmcl_embree)

    add_definitions(-DRMCL_EMBREE)
    set(RMCL_EMBREE TRUE)

endif(TARGET rmagine::embree)

# CUDA
if(TARGET rmagine::cuda AND NOT ${DISABLE_CUDA})
    # Build cuda related stuff
    find_package(CUDA)

    enable_language(CUDA)
    include_directories(${CUDA_INCLUDE_DIRS})

    # include(FindCUDA/select_compute_arch)
    cuda_detect_installed_gpus(INSTALLED_GPU_CCS_1)
    string(STRIP "${INSTALLED_GPU_CCS_1}" INSTALLED_GPU_CCS_2)
    string(REPLACE " " ";" INSTALLED_GPU_CCS_3 "${INSTALLED_GPU_CCS_2}")
    string(REPLACE "." "" CUDA_ARCH_LIST "${INSTALLED_GPU_CCS_3}")
    # message(WARNING "CUDA_ARCH_LIST: ${CUDA_ARCH_LIST}")
    # SET(CMAKE_CUDA_ARCHITECTURES ${CUDA_ARCH_LIST})

    add_library(rmcl_cuda
        # math
        src/rmcl/math/math_batched.cu
        src/rmcl/math/math.cu
    )

    set_property(TARGET rmcl_cuda
        PROPERTY "${CUDA_ARCH_LIST}"
    )

    target_link_libraries(rmcl_cuda
        rmagine::cuda
        ${CUDA_LIBRARIES}
    )

    install(TARGETS rmcl_cuda
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
    )

    list(APPEND RMCL_LIBS rmcl_cuda)

    add_definitions(-DRMCL_CUDA)
    set(RMCL_CUDA True)
endif(TARGET rmagine::cuda AND NOT ${DISABLE_CUDA})

if(TARGET rmagine::optix AND NOT ${DISABLE_OPTIX})
    # Build optix related stuff

    # message(WARNING "CMAKE_BINARY_DIR: ${CMAKE_BINARY_DIR}")
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

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

    # TODO: move this to rmagine
    get_target_property(RMAGINE_OPTIX_INCLUDES rmagine::optix INTERFACE_INCLUDE_DIRECTORIES)

    cuda_include_directories(
        ${RMAGINE_OPTIX_INCLUDES}
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

    add_dependencies(rmcl_optix
        rmcl_optix_ptx
    )

    target_include_directories(rmcl_optix PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
    )

    target_link_libraries(rmcl_optix
        rmcl
        rmcl_cuda
        rmagine::optix
    )

    install(TARGETS rmcl_optix
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
    )

    list(APPEND RMCL_LIBS rmcl_optix)

    add_definitions(-DRMCL_OPTIX)
    set(RMCL_OPTIX True)
endif(TARGET rmagine::optix AND NOT ${DISABLE_OPTIX})


install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

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

target_link_libraries(micp_localization
    rmcl_ros
)

if(RMCL_EMBREE)
    target_link_libraries(micp_localization
        rmcl_embree
    )
endif(RMCL_EMBREE)
    
if(RMCL_CUDA)
    target_link_libraries(micp_localization
        rmcl_cuda
    )
endif(RMCL_CUDA)

if(RMCL_OPTIX)
    target_link_libraries(micp_localization
        rmcl_optix
    )
endif(RMCL_OPTIX)

ament_target_dependencies(micp_localization
    rclcpp
    geometry_msgs
    sensor_msgs
    tf2_ros
    rmcl_msgs
    image_transport
    visualization_msgs
)

install(TARGETS 
    micp_localization
  DESTINATION lib/${PROJECT_NAME})


# TODO: PORT TO ROS2!
# ########################
# ### CONVERSION NODES ###
# ########################
# if(BUILD_CONV)

#     ####### PCL2 to SCAN CONVERTER
#     add_executable(conv_pcl2_to_scan src/nodes/conv/pcl2_to_scan.cpp)

#     add_dependencies(conv_pcl2_to_scan 
#         ${${PROJECT_NAME}_EXPORTED_TARGETS}
#         ${catkin_EXPORTED_TARGETS}
#     )

#     ## Specify libraries to link a library or executable target against
#     target_link_libraries(conv_pcl2_to_scan
#         ${catkin_LIBRARIES}
#         rmcl_ros
#     )

#     ####### PCL2 to DEPTH CONVERTER
#     add_executable(conv_pcl2_to_depth src/nodes/conv/pcl2_to_depth.cpp)

#     add_dependencies(conv_pcl2_to_depth 
#         ${${PROJECT_NAME}_EXPORTED_TARGETS}
#         ${catkin_EXPORTED_TARGETS}
#     )

#     ## Specify libraries to link a library or executable target against
#     target_link_libraries(conv_pcl2_to_depth
#         ${catkin_LIBRARIES}
#         ${rmagine_ext_LIBRARIES}
#         rmcl_ros
#     )

#     ####### IMAGE to DEPTH CONVERTER
#     add_executable(conv_image_to_depth src/nodes/conv/image_to_depth.cpp)

#     add_dependencies(conv_image_to_depth 
#         ${${PROJECT_NAME}_EXPORTED_TARGETS}
#         ${catkin_EXPORTED_TARGETS}
#     )

#     ## Specify libraries to link a library or executable target against
#     target_link_libraries(conv_image_to_depth
#         ${catkin_LIBRARIES}
#         rmcl_ros
#     )

# endif(BUILD_CONV)

# ################
# ### EXAMPLES ###
# ################
if(BUILD_MICP_EXPERIMENTS)

if(RMCL_EMBREE)

### EVAL ###
add_executable(micp_eval src/nodes/examples/micp_eval.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(micp_eval
    rmcl_embree
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

endif(RMCL_EMBREE)

endif(BUILD_MICP_EXPERIMENTS)

ament_export_include_directories(
  include
)

ament_export_libraries(
  ${RMCL_LIBS}
)

ament_export_dependencies(rclcpp
  geometry_msgs
  sensor_msgs
  tf2_ros
  rmcl_msgs
  image_transport
  visualization_msgs)

ament_package()