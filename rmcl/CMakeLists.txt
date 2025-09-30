cmake_minimum_required(VERSION 3.16)
project(rmcl
    LANGUAGES CXX C
	VERSION 2.2.3
	)

option(BUILD_TESTS "Build tests" ON)

# This gives an compiler error for standard ubuntu 20 setup including
# - ROS noetic
# - CUDA 11.4
# - cmake 3.16.3
# 
# Error Message:
# """
# Errors     << rmcl:check /home/amock/workspaces/ros/warpsense_ws/logs/rmcl/build.check.022.log                                                      
# CMake Error at /home/amock/workspaces/ros/warpsense_ws/src/rmcl_stack/rmcl/CMakeLists.txt:5 (cmake_policy):
# Policy "CMP0148" is not known to this version of CMake.
# CMake Error at /home/amock/workspaces/ros/warpsense_ws/src/rmcl_stack/rmcl/CMakeLists.txt:6 (cmake_policy):
# Policy "CMP0146" is not known to this version of CMake.
# """
# 
# before merging this into the noetic branch we need to make sure this compiles on standard ubuntu / ROS noetic systems
# 
# cmake_policy(SET CMP0148 OLD)
# cmake_policy(SET CMP0146 OLD)
#

## Compile as C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# DEFAULT RELEASE
if (NOT EXISTS ${CMAKE_BINARY_DIR}/CMakeCache.txt)
  if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
  endif()
endif()

set_property(GLOBAL PROPERTY USE_FOLDERS On)
set_property(GLOBAL PROPERTY FIND_LIBRARY_USE_LIB64_PATHS ON)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(GNUInstallDirs)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_TMP_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/tmp)

list(APPEND CMAKE_MODULE_PATH ${rmcl_SOURCE_DIR}/cmake)


find_package(TBB REQUIRED)
if(TARGET TBB::tbb)
    set(TBB_INCLUDE_DIRS "")
    set(TBB_LIBRARIES TBB::tbb)
endif()

# only print warning for Rmagine version greater than RMAGINE_MAX_VERSION
set(RMAGINE_MAX_VERSION "2.4.0")

find_package(rmagine 2.3.2
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


if(TARGET rmagine::embree)
    option(DISABLE_EMBREE "Disable Rmagine Embree backend Compilation" FALSE)
endif()

if(TARGET rmagine::cuda)
    option(DISABLE_CUDA "Disable CUDA related compilations" FALSE)
    if(CMAKE_CUDA_COMPILER)
        enable_language(CUDA)
    endif(CMAKE_CUDA_COMPILER)
endif()

if(TARGET rmagine::optix)
    option(DISABLE_OPTIX "Disable Rmagine OptiX backend Compilation" FALSE)
endif()

find_package(Eigen3 REQUIRED)

set(RMCL_LIBRARIES rmcl)

if(TARGET rmagine::embree AND NOT ${DISABLE_EMBREE})
    list(APPEND RMCL_LIBRARIES rmcl-embree)
endif()

if(TARGET rmagine::cuda AND NOT ${DISABLE_CUDA})
    list(APPEND RMCL_LIBRARIES rmcl-cuda)
endif()

if(TARGET rmagine::optix AND NOT ${DISABLE_OPTIX})
    list(APPEND RMCL_LIBRARIES rmcl-optix)
endif()

include_directories(
    include
)

# CORE LIB
add_library(rmcl SHARED
    # Registration
    src/rmcl/registration/CorrespondencesCPU.cpp
)

# add_dependencies(rmcl nanoflann)
target_link_libraries(rmcl
    rmagine::core
    Eigen3::Eigen
)

target_include_directories(rmcl
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/rmcl-${rmcl_VERSION}>
)

set_target_properties(rmcl
    PROPERTIES
      SOVERSION ${rmcl_VERSION_MAJOR}
      VERSION ${rmcl_VERSION}
      # CXX_STANDARD 17
)

# EMBREE
if(TARGET rmagine::embree)

    add_library(rmcl-embree SHARED
        # Correction
        src/rmcl/registration/RCCEmbree.cpp
        src/rmcl/registration/CPCEmbree.cpp
    )

    target_link_libraries(rmcl-embree
        rmagine::core
        rmagine::embree
        rmcl
        ${TBB_LIBRARIES}
    )

    target_include_directories(rmcl-embree
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/rmcl-${rmcl_VERSION}>
        ${TBB_INCLUDE_DIRS}
    )

    set_target_properties(rmcl-embree
        PROPERTIES
            SOVERSION ${rmcl_VERSION_MAJOR}
            VERSION ${rmcl_VERSION}
            # CXX_STANDARD 17
    )

    install(TARGETS rmcl-embree
        EXPORT rmcl-targets
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    )

    add_definitions(-DRMCL_EMBREE)
    set(RMCL_EMBREE TRUE)
endif()

if(TARGET rmagine::cuda OR TARGET rmagine::optix)

    # copied from rmagine:
    # I am aware find_package(CUDA) is deprecated.
    # - TODO: support find_package(CUDAToolkit) as well
    # -- Example: https://github.com/ceres-solver/ceres-solver/blob/master/CMakeLists.txt
    # - TODO: this becomes quite messy. make a seperate file from this
    # Or wait until all targeted platforms use `find_package(CUDAToolkit)`, then remove everything

    # default flags are not set when including cuda
    if (NOT EXISTS ${CMAKE_BINARY_DIR}/CMakeCache.txt)
        if(NOT CUDA_NVCC_FLAGS_DEBUG)
            set(CUDA_NVCC_FLAGS_DEBUG "-g" CACHE STRING "" FORCE)
        endif()
        if(NOT CUDA_NVCC_FLAGS_MINSIZEREL)
            set(CUDA_NVCC_FLAGS_MINSIZEREL "-Os -DNDEBUG" CACHE STRING "" FORCE)
        endif()
        if(NOT CUDA_NVCC_FLAGS_RELEASE)
            set(CUDA_NVCC_FLAGS_RELEASE "-O3 -DNDEBUG" CACHE STRING "" FORCE)
        endif()
        if(NOT CUDA_NVCC_FLAGS_RELWITHDEBINFO)
            set(CUDA_NVCC_FLAGS_RELWITHDEBINFO "-O2 -g -DNDEBUG" CACHE STRING "" FORCE)
        endif()
    endif()

    include(CheckLanguage)
    check_language(CUDA)
    if(CMAKE_CUDA_COMPILER)
        message(STATUS "Cuda language available!")

        find_package(CUDAToolkit QUIET)
        if(CUDAToolkit_FOUND)
            message(STATUS "Found Cuda Toolkit!")
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
                message(STATUS "Found Cuda!")
                enable_language(CUDA)
                set(CUDA_DRIVER_LIBRARY cuda)
            else()
                message(STATUS "Neither CudaToolkit nor CUDA found!")
            endif(CUDA_FOUND)
        endif(CUDAToolkit_FOUND)
    endif(CMAKE_CUDA_COMPILER)
endif()

# CUDA
if(TARGET rmagine::cuda AND NOT ${DISABLE_CUDA})
    add_library(rmcl-cuda SHARED
        src/rmcl/registration/CorrespondencesCUDA.cpp
    )

    target_link_libraries(rmcl-cuda
        rmagine::cuda
        ${CUDA_LIBRARIES}
    )

    target_include_directories(rmcl-cuda
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/rmcl-${rmcl_VERSION}>
    )

    set_target_properties(rmcl-cuda
        PROPERTIES
            SOVERSION ${rmcl_VERSION_MAJOR}
            VERSION ${rmcl_VERSION}
            # CXX_STANDARD 17
    )

    install(TARGETS rmcl-cuda
        EXPORT rmcl-targets
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    )

    add_definitions(-DRMCL_CUDA)
    set(RMCL_CUDA True)
endif()


# CUDA
if(TARGET rmagine::optix AND NOT ${DISABLE_OPTIX})
    add_library(rmcl-optix SHARED
        # math
        src/rmcl/registration/RCCOptix.cpp
    )

    target_link_libraries(rmcl-optix
        rmagine::optix
        ${CUDA_LIBRARIES}
    )

    target_include_directories(rmcl-optix
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/rmcl-${rmcl_VERSION}>
    )

    set_target_properties(rmcl-optix
        PROPERTIES
            SOVERSION ${rmcl_VERSION_MAJOR}
            VERSION ${rmcl_VERSION}
            # CXX_STANDARD 17
    )

    install(TARGETS rmcl-optix
        EXPORT rmcl-targets
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    )

    add_definitions(-DRMCL_OPTIX)
    set(RMCL_OPTIX True)
endif()

### INSTALL
include(CMakePackageConfigHelpers)

install(TARGETS rmcl 
  EXPORT rmcl-targets
)

install(EXPORT rmcl-targets
    FILE rmcl-targets.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/rmcl-${rmcl_VERSION}
)

write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/rmcl-config-version.cmake
    VERSION ${rmcl_VERSION}
    COMPATIBILITY SameMajorVersion
)

configure_package_config_file(cmake/rmcl-config.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/rmcl-config.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/rmcl-${rmcl_VERSION}
)

## INSTALL
install(
    FILES 
        ${CMAKE_CURRENT_BINARY_DIR}/rmcl-config.cmake
        ${CMAKE_CURRENT_BINARY_DIR}/rmcl-config-version.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/rmcl-${rmcl_VERSION}
)

# HEADERS
install(
  DIRECTORY include/rmcl
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/rmcl-${rmcl_VERSION}
)

install(FILES package.xml
    DESTINATION share/rmcl-${rmcl_VERSION}
)

