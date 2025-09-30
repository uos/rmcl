cmake_minimum_required(VERSION 3.18)
project(rmcl_ros
	VERSION 2.2.3)

# option(RMCL_ROS_BUILD_EXPERIMENTAL "Build Experimental Code" OFF)
# option(RMCL_ROS_BUILD_MICP_EXPERIMENTS "Build Experiments" OFF)


# To still use FetchContent_Populate for non CMake projects 
# without triggering a warning
if(POLICY CMP0169)
    cmake_policy(SET CMP0169 OLD)
endif()

include(FetchContent)


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

# DEFAULT RELEASE
if (NOT EXISTS ${CMAKE_BINARY_DIR}/CMakeCache.txt)
  if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
  endif()
endif()

list(APPEND CMAKE_MODULE_PATH ${rmcl_ros_SOURCE_DIR}/cmake)

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rmcl_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_srvs REQUIRED)


find_package(rmagine 2.3.1
    COMPONENTS
        core
    OPTIONAL_COMPONENTS
        embree
        cuda
        optix
)

find_package(rmcl 2.2.3 REQUIRED)
include_directories(${rmcl_INCLUDE_DIR})

# TODO: we can do this better. maybe put definitions to each target?
if(TARGET rmcl-embree)
    add_definitions(-DRMCL_EMBREE)
    set(RMCL_EMBREE TRUE)
endif(TARGET rmcl-embree)

if(TARGET rmcl-cuda)
    add_definitions(-DRMCL_CUDA)
    set(RMCL_CUDA True)
endif(TARGET rmcl-cuda)

if(TARGET rmcl-optix)
    add_definitions(-DRMCL_OPTIX)
    set(RMCL_OPTIX True)
endif(TARGET rmcl-optix)


include_directories(
    include
)

# CORE LIB ROS
add_library(rmcl_ros SHARED
    src/util/conversions.cpp
    src/util/scan_operations.cpp
    src/util/ros_helper.cpp
    # micp localization
    src/micpl/MICPSensor.cpp
    src/micpl/MICPSensorCPU.cpp
    src/micpl/MICPSphericalSensorCPU.cpp
    src/micpl/MICPPinholeSensorCPU.cpp
    src/micpl/MICPO1DnSensorCPU.cpp
    src/micpl/MICPOnDnSensorCPU.cpp
    # rmcl localization
    src/rmcl/KLDResamplerCPU.cpp
    src/rmcl/ResidualResamplerCPU.cpp
    src/rmcl/GladiatorResamplerCPU.cpp
)

target_include_directories(rmcl_ros PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

target_link_libraries(rmcl_ros
    rmcl
    rmagine::core
)

ament_target_dependencies(rmcl_ros
    rclcpp
    sensor_msgs
    rmcl_msgs
    geometry_msgs
    tf2_ros
    visualization_msgs
    std_srvs
)

install(TARGETS rmcl_ros
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

if(TARGET rmcl-embree)

    add_library(rmcl_embree_ros SHARED
        # rmcl localization
        src/rmcl/TFMotionUpdaterCPU.cpp
        src/rmcl/PCDSensorUpdaterEmbree.cpp
    )

    target_include_directories(rmcl_embree_ros PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

    target_link_libraries(rmcl_embree_ros
        rmcl-embree
        rmagine::core
        rmagine::embree
    )

    ament_target_dependencies(rmcl_embree_ros
        rclcpp
        sensor_msgs
        rmcl_msgs
        geometry_msgs
        tf2_ros
        visualization_msgs
        std_srvs
    )

    install(TARGETS rmcl_embree_ros
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
    )

endif(TARGET rmcl-embree)

if(RMCL_CUDA)

    set(RMCL_OPTIX_DOWNLOAD_VERSION "9.0.0" CACHE STRING "Desired optix version used for download.")

    include(CheckLanguage)
    check_language(CUDA)
    if(CMAKE_CUDA_COMPILER)
        find_package(CUDAToolkit QUIET)
        if(CUDAToolkit_FOUND)
            enable_language(CUDA)
            set(CUDA_FOUND True)
            set(CUDA_LIBRARIES CUDA::cudart)
            set(CUDA_cusolver_LIBRARY CUDA::cusolver)
            set(CUDA_cublas_LIBRARY CUDA::cublas)
            set(CUDA_DRIVER_LIBRARY CUDA::cuda_driver)
            set(CUDA_INCLUDE_DIRS "") # is in target instead
        else()
            find_package(CUDA)
            if(CUDA_FOUND)
                enable_language(CUDA)
                # 1. CUDA_DRIVER_LIBRARY <- Figure out if there is a cuda driver library available
                if(CUDA_CUDA_LIBRARY)
                    set(CUDA_DRIVER_LIBRARY ${CUDA_CUDA_LIBRARY})
                else()
                    find_library(CUDA_DRIVER_LIBRARY NAMES cuda)
                endif()
            else()
                message(STATUS "Neither CudaToolkit nor CUDA found!")
            endif(CUDA_FOUND)
        endif(CUDAToolkit_FOUND)
    endif(CMAKE_CUDA_COMPILER) # -> CUDA_FOUND

    add_library(rmcl_ros_cuda SHARED
        # MICP-L
        src/micpl/MICPSensorCUDA.cpp
        src/micpl/MICPSphericalSensorCUDA.cpp
        src/micpl/MICPPinholeSensorCUDA.cpp
        src/micpl/MICPO1DnSensorCUDA.cpp
        src/micpl/MICPOnDnSensorCUDA.cpp
        # RMCL
        src/rmcl/particle_motion.cu
        src/rmcl/resampling.cu
        src/rmcl/GladiatorResamplerGPU.cpp
    )

    target_include_directories(rmcl_ros_cuda PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    )
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
        tf2_ros
        std_srvs
    )

    install(TARGETS rmcl_ros_cuda
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
    )

    if(TARGET rmcl-optix)

        # A target rmcl-optix: Assumption. OptiX must be available on the system.

        # Replace this

        find_package(OptiX QUIET)

        if(NOT OptiX_FOUND AND OptiX_LIBRARY_FOUND) # No headers found but library available

            if(OptiX_LIBRARY_DRIVER_VERSION)
                message(STATUS "Auto-detected NVIDIA driver version (from OptiX lib): ${OptiX_LIBRARY_DRIVER_VERSION}")
                include(MaxOptixVersion)
                max_optix_version(OptiX_LIBRARY_DRIVER_VERSION MAX_OPTIX_VERSION)

                if(MAX_OPTIX_VERSION)
                    if(${RMCL_OPTIX_DOWNLOAD_VERSION} VERSION_GREATER ${MAX_OPTIX_VERSION})
                        message(STATUS "Decrease RMCL_OPTIX_DOWNLOAD_VERSION from ${RMCL_OPTIX_DOWNLOAD_VERSION} to ${MAX_OPTIX_VERSION}. This is the latest supported version for the currently used NVIDIA driver version ${NVIDIA_DRIVER_VERSION}")
                        set(RMCL_OPTIX_DOWNLOAD_VERSION ${MAX_OPTIX_VERSION})
                    endif()
                else(MAX_OPTIX_VERSION)
                    message(WARNING "Could not determine maximum allowed OptiX version for NVIDIA driver ${OptiX_LIBRARY_DRIVER_VERSION}. Trying ${RMCL_OPTIX_DOWNLOAD_VERSION}")
                endif(MAX_OPTIX_VERSION)

            endif(OptiX_LIBRARY_DRIVER_VERSION)

            message(STATUS "Searching for OptiX online")

            FetchContent_Declare(
                optix_header
                GIT_REPOSITORY   https://github.com/NVIDIA/optix-dev.git
                GIT_TAG          v${RMCL_OPTIX_DOWNLOAD_VERSION}
            )
            FetchContent_Populate(optix_header)

            set(OptiX_ROOT_DIR ${optix_header_SOURCE_DIR})
            find_package(OptiX QUIET)
        endif()

        if(NOT OptiX_FOUND)
            message(FATAL_ERROR "CMake error in finding OptiX (rmcl_ros). This should be impossible, since rmcl-optix target was already built.")
        endif(NOT OptiX_FOUND)

        set(RMCL_ROS_OPTIX_PTX_DIR "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/rmcl_ros_optix_ptx")
        set(RMCL_ROS_OPTIX_PTX_GLOB_DIR "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}/rmcl_ros_optix_ptx")

        set(CUDA_GENERATED_OUTPUT_DIR ${RMCL_ROS_OPTIX_PTX_DIR})

        message(STATUS "Writing Optix Kernels to ${RMCL_ROS_OPTIX_PTX_DIR}")

        set(OPTIX_KERNEL_FILES
            src/rmcl/optix/BeamEvaluateProgram.cu
        )

        if(CUDAToolkit_FOUND)
            # NEW VERSION

            add_library(rmcl_ros_optix_cu_to_ptx OBJECT
                ${OPTIX_KERNEL_FILES}
            )

            target_include_directories(rmcl_ros_optix_cu_to_ptx PRIVATE
                $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                $<BUILD_INTERFACE:${OptiX_INCLUDE_DIRS}>
            )

            set_target_properties(rmcl_ros_optix_cu_to_ptx 
                PROPERTIES
                    CUDA_PTX_COMPILATION ON
                    # CUDA_ARCHITECTURES all
            )

            target_link_libraries(rmcl_ros_optix_cu_to_ptx
                ${OptiX_LIBRARIES}
                rmagine::core
                rmagine::cuda
            )

            add_custom_target(rmcl_ros_optix_ptx ALL
                DEPENDS 
                    rmagine::core 
                    rmcl_ros_optix_cu_to_ptx 
                    ${OPTIX_KERNEL_FILES}
                SOURCES ${OPTIX_KERNEL_FILES}
                VERBATIM)

            add_custom_command(
                TARGET rmcl_ros_optix_ptx POST_BUILD
                COMMAND ${CMAKE_COMMAND} 
                    -DRMCL_ROS_SOURCE_DIR=${rmcl_ros_SOURCE_DIR} 
                    -DRMCL_ROS_OPTIX_PTX_DIR=${RMCL_ROS_OPTIX_PTX_DIR}
                    -DOPTIX_KERNEL_FILES="${OPTIX_KERNEL_FILES}"
                    -P "${CMAKE_CURRENT_LIST_DIR}/cmake/CompileOptixKernelsCudaToolkit.cmake"
            )
        else(CUDAToolkit_FOUND)
            # THIS IS GOING TO BE OBSOLETE

            cuda_include_directories(
                ${OptiX_INCLUDE_DIRS}
            )

            cuda_compile_ptx(RMCL_ROS_OPTIX_PTX_FILES
                ${OPTIX_KERNEL_FILES}
            )

            add_custom_target(rmcl_ros_optix_ptx ALL
                DEPENDS
                    ${RMCL_ROS_OPTIX_PTX_FILES}
                    ${OPTIX_KERNEL_FILES}
                SOURCES ${OPTIX_KERNEL_FILES}
                VERBATIM)

            message(STATUS "CMAKE_SOURCE_DIR: ${rmcl_ros_SOURCE_DIR}")

            add_custom_command(
                TARGET rmcl_ros_optix_ptx POST_BUILD
                COMMAND ${CMAKE_COMMAND} 
                    -DRMCL_ROS_SOURCE_DIR=${rmcl_ros_SOURCE_DIR} 
                    -DRMCL_ROS_OPTIX_PTX_DIR=${RMCL_ROS_OPTIX_PTX_DIR} 
                    -DOPTIX_KERNEL_FILES="${OPTIX_KERNEL_FILES}" 
                    -P "${CMAKE_CURRENT_LIST_DIR}/cmake/CompileOptixKernels.cmake"
            )
        endif(CUDAToolkit_FOUND)


        add_library(rmcl_ros_optix SHARED
            src/rmcl/TFMotionUpdaterGPU.cpp
            src/rmcl/PCDSensorUpdaterOptix.cpp
            # Correction Programs
            src/rmcl/optix/eval_modules.cpp
            src/rmcl/optix/eval_program_groups.cpp
            src/rmcl/optix/eval_pipelines.cpp
        )

        add_dependencies(rmcl_ros_optix
            rmcl_ros_optix_ptx
        )

        target_include_directories(rmcl_ros_optix PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
            $<BUILD_INTERFACE:${OptiX_INCLUDE_DIRS}>
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:include>)

        target_link_libraries(rmcl_ros_optix
            rmcl
            rmcl_ros
            rmcl-cuda
            rmcl_ros_cuda
            rmcl-optix
            rmagine::optix
        )

        ament_target_dependencies(rmcl_ros_optix
            rclcpp
            sensor_msgs
            rmcl_msgs
            geometry_msgs
            tf2_ros
            std_srvs
        )

        install(TARGETS rmcl_ros_optix
            ARCHIVE DESTINATION lib
            LIBRARY DESTINATION lib
            RUNTIME DESTINATION bin
        )
    endif(TARGET rmcl-optix)

endif(RMCL_CUDA)

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
    rmagine::core
)

if(RMCL_EMBREE)
    target_link_libraries(micp_localization
        rmcl-embree
    )
endif(RMCL_EMBREE)
    
if(RMCL_CUDA)
    target_link_libraries(micp_localization
        rmcl_ros_cuda
    )
endif(RMCL_CUDA)

if(RMCL_OPTIX)
    target_link_libraries(micp_localization
        rmcl-optix
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
    std_srvs
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
    std_srvs
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
    std_srvs
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







#########
# FILTER / SEGMENTATION
#####
add_library(map_segmentation SHARED
    src/nodes/filter/map_segmentation.cpp
)

ament_target_dependencies(map_segmentation
    rclcpp
    rclcpp_components
    geometry_msgs
    sensor_msgs
    tf2
    tf2_ros
    rmcl_msgs
    image_transport
    visualization_msgs
    std_srvs
)

install(TARGETS 
    map_segmentation
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

# TODO: add more segmentation nodes
if(TARGET rmcl-embree)

add_library(o1dn_map_segmentation_embree SHARED
    src/nodes/filter/o1dn_map_segmentation_embree.cpp)

target_link_libraries(o1dn_map_segmentation_embree
    rmcl_ros
    rmcl-embree
    map_segmentation
)

ament_target_dependencies(o1dn_map_segmentation_embree
    rclcpp
    rclcpp_components
    geometry_msgs
    sensor_msgs
    tf2_ros
    rmcl_msgs
    std_srvs
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
    src/nodes/filter/o1dn_map_segmentation_embree.cpp)

target_link_libraries(scan_map_segmentation_embree
    rmcl_ros
    rmcl-embree
    map_segmentation
)

ament_target_dependencies(scan_map_segmentation_embree
    rclcpp
    rclcpp_components
    geometry_msgs
    sensor_msgs
    tf2_ros
    rmcl_msgs
    std_srvs
)

rclcpp_components_register_node(scan_map_segmentation_embree 
    PLUGIN "rmcl::ScanMapSegmentationEmbreeNode" 
    EXECUTABLE scan_map_segmentation_embree_node)

install(TARGETS 
    scan_map_segmentation_embree
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin      
)

endif(TARGET rmcl-embree)






###################
#### RMCL Node ####
###################

if(RMCL_OPTIX AND RMCL_EMBREE)

# The first prototype (v0) for global localization will only compile when both and optix and embree is available.
# So only for NVIDIA devices it is possible to 
# Next step is to make things 

add_library(rmcl_localization SHARED
  src/nodes/rmcl_localization.cpp)

target_include_directories(rmcl_localization
  PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

target_link_libraries(rmcl_localization
    rmcl_ros
    Eigen3::Eigen
    rmagine::core
    rmagine::embree
)

if(RMCL_EMBREE)
    target_link_libraries(rmcl_localization
        rmcl-embree
        rmcl_embree_ros
    )
endif(RMCL_EMBREE)
    
if(RMCL_CUDA)
    target_link_libraries(rmcl_localization
        rmcl-cuda
    )
endif(RMCL_CUDA)

if(RMCL_OPTIX)
    message(WARNING "Linking CUDA + OptiX related code")
    target_link_libraries(rmcl_localization
        rmcl_ros_cuda
        rmcl-cuda
        rmcl_ros_optix
        rmcl-optix
    )
endif(RMCL_OPTIX)

ament_target_dependencies(rmcl_localization 
    rclcpp
    rclcpp_components
    geometry_msgs
    sensor_msgs
    tf2_ros
    visualization_msgs
)

rclcpp_components_register_node(rmcl_localization PLUGIN "rmcl::RmclNode" EXECUTABLE rmcl_localization_node)

install(TARGETS 
    rmcl_localization
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

endif()


#################
# Finish package configuration

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
  visualization_msgs
  std_srvs)

ament_package()
