cmake_minimum_required(VERSION 3.16)
project(rmcl
    LANGUAGES CXX C
	VERSION 1.2.0
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



# message(STATUS "CMake Version: ${CMAKE_VERSION}")
# if(CMAKE_VERSION VERSION_GREATER_EQUAL 3.9)
#     message(STATUS ">= 3.9 - Enabling Link Time Optimization")
#     set(CMAKE_INTERPROCEDURAL_OPTIMIZATION TRUE)
# endif()

list(APPEND CMAKE_MODULE_PATH ${rmcl_SOURCE_DIR}/cmake)

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

### EXTERNAL PROJECTS
include(ExternalProject)

##########################################
## NanoFLANN: K-nearest neighbor search ##
##########################################
include(DownloadNanoflann)
include_directories(${nanoflann_INCLUDE_DIR})

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
    # Math
    src/rmcl/math/math.cpp
    src/rmcl/math/math_batched.cpp
    # # Spatial
    # src/rmcl/spatial/KdTree.cpp # TODO: fix nanoflann
    # # Clustering
    # src/rmcl/clustering/clustering.cpp # TODO: fix nanoflann
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

# install(TARGETS rmcl
#     EXPORT rmcl-targets
#     ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
#     LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
# )

# EMBREE
if(TARGET rmagine::embree)

    add_library(rmcl-embree SHARED
        # Correction
        src/rmcl/correction/SphereCorrectorEmbree.cpp
        src/rmcl/correction/PinholeCorrectorEmbree.cpp
        src/rmcl/correction/O1DnCorrectorEmbree.cpp
        src/rmcl/correction/OnDnCorrectorEmbree.cpp
    )

    target_link_libraries(rmcl-embree
        rmagine::core
        rmagine::embree
        ${rmagine_ext_LIBRARIES}
        rmcl
    )

    target_include_directories(rmcl-embree
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/rmcl-${rmcl_VERSION}>
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

# CUDA
if(TARGET rmagine::cuda AND NOT ${DISABLE_CUDA})
    add_library(rmcl-cuda SHARED
        # math
        src/rmcl/math/math_batched.cu
        src/rmcl/math/math.cu
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

if(TARGET rmagine::optix AND NOT ${DISABLE_OPTIX})
    # Build optix related stuff
    find_package(CUDA)

    enable_language(CUDA)

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

    add_library(rmcl-optix SHARED
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

    add_dependencies(rmcl-optix
        rmcl_optix_ptx
    )

    set_target_properties(rmcl-optix
        PROPERTIES
            SOVERSION ${rmcl_VERSION_MAJOR}
            VERSION ${rmcl_VERSION}
            # CXX_STANDARD 17
    )

    target_include_directories(rmcl-optix PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/rmcl-${rmcl_VERSION}>
    )

    target_link_libraries(rmcl-optix
        rmcl
        rmcl-cuda
        rmagine::optix
    )

    install(TARGETS rmcl-optix
        EXPORT rmcl-targets
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    )

    add_definitions(-DRMCL_OPTIX)
    set(RMCL_OPTIX True)
endif()


#### TESTS
if(BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif(BUILD_TESTS)


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

