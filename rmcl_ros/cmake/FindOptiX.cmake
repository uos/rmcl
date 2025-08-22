### FindOptiX
# Variables Set:
# Pure Optix:
# - OptiX_LIBRARY (Path to optix library)
# - OptiX_LIBRARY_DRIVER_VERSION (NVIDIA driver version for optix library, fetched from library name)
# - OptiX_INCLUDE_DIR (Path to OptiX header folder)
# - OptiX_VERSION (fetched from headers)
# - OptiX_FOUND (= TRUE, only if header & lib is found) 
# Additional:
# - OptiX_INCLUDE_DIRS
# - Optix_LIBRARIES

find_package(PkgConfig)
pkg_check_modules(OptiX QUIET optix)

# 1. LIBRARY

# 1. general search
find_library(OptiX_LIBRARY
  NAMES optix.1 optix
  )

# 2. more special search
if(NOT OptiX_LIBRARY)
find_library(OptiX_LIBRARY
  NAMES nvoptix.1 nvoptix libnvoptix.so.1 libnvoptix.so
  )
endif()
# 3. more special search
if(NOT OptiX_LIBRARY)
find_library(OptiX_LIBRARY
  NAMES libnvoptix.so.1 libnvoptix.so
  )
endif()

if(OptiX_LIBRARY)
  set(OptiX_LIBRARY_FOUND TRUE)

  # try to auto-determine driver version
  get_filename_component(OptiX_LIBRARY_REALPATH ${OptiX_LIBRARY} REALPATH)
  get_filename_component(OptiX_LIBRARY_REALNAME ${OptiX_LIBRARY_REALPATH} NAME)

  # Try extracting version number from filename
  string(REGEX MATCH "[0-9]+(\\.[0-9]+)*" OptiX_LIBRARY_DRIVER_VERSION "${OptiX_LIBRARY_REALNAME}")

  message(STATUS "OptiX Driver version: ${OptiX_LIBRARY_DRIVER_VERSION}")

endif(OptiX_LIBRARY)

# 2. HEADERS

# set(OptiX_ROOT_DIR "" CACHE PATH "Root of Optix installation")

if(DEFINED ENV{OPTIX_INCLUDE_DIR})
  message(STATUS "FindOptiX: Found path to optix headers in environment variable 'OPTIX_INCLUDE_DIR'. ")
  set(OptiX_INCLUDE_DIR_ENV "$ENV{OPTIX_INCLUDE_DIR}")

  # INCLUDE
  find_path(OptiX_INCLUDE_DIR
    NAMES optix.h
    PATHS "${OptiX_INCLUDE_DIR_ENV}"
    NO_DEFAULT_PATH
  )

  if(NOT OptiX_INCLUDE_DIR)
  message(WARNING "Environment variable 'OptiX_INCLUDE_DIR' pointing to optix headers folder that is not conform. Make sure it does contain the 'optix.h' file.")
  endif(NOT OptiX_INCLUDE_DIR)

endif(DEFINED ENV{OPTIX_INCLUDE_DIR})

# INCLUDE
find_path(OptiX_INCLUDE_DIR
  NAMES optix.h
  PATHS "${OptiX_ROOT_DIR}/include"
  NO_DEFAULT_PATH
)

if(NOT OptiX_INCLUDE_DIR)
find_path(OptiX_INCLUDE_DIR
  NAMES optix.h
)
endif()

if(NOT OptiX_INCLUDE_DIR)
find_path(OptiX_INCLUDE_DIR
  NAMES optix.h
  PATHS "/opt/optix/include"
  NO_DEFAULT_PATH
)
endif()

if(NOT OptiX_INCLUDE_DIR)
find_path(OptiX_INCLUDE_DIR
  NAMES optix.h
  PATHS "~/optix/include"
  NO_DEFAULT_PATH
)
endif()


# VERSION
if(OptiX_INCLUDE_DIR)

  set(OptiX_HEADERS_FOUND TRUE)

  file(READ "${OptiX_INCLUDE_DIR}/optix.h" OPTIX_H)
  string(REGEX MATCH "OPTIX_VERSION[ \t\r\n\\]+([0-9]+)" _ ${OPTIX_H})
  set(OptiX_VERSION_RAW ${CMAKE_MATCH_1})


  if(OptiX_VERSION_RAW)
    math(EXPR OptiX_VERSION_MAJOR "${OptiX_VERSION_RAW} / 10000")
    math(EXPR OptiX_VERSION_MINOR "(${OptiX_VERSION_RAW} % 10000) / 100")
    math(EXPR OptiX_VERSION_MICRO "${OptiX_VERSION_RAW} % 100")

    set(OptiX_VERSION_PATCH ${OptiX_VERSION_MICRO})
    set(OptiX_VERSION "${OptiX_VERSION_MAJOR}.${OptiX_VERSION_MINOR}.${OptiX_VERSION_MICRO}")
  else()
    message(WARNING "Could not find OptiX version definition in optix.h")
  endif()

endif()


# 3. Cleanup 
if(OptiX_LIBRARY AND OptiX_INCLUDE_DIR)
  set(OptiX_FOUND TRUE)
  set(OptiX_LIBRARIES ${OptiX_LIBRARY})
  set(OptiX_INCLUDE_DIRS ${OptiX_INCLUDE_DIR})

  message(STATUS "OPTIX LIB: ${OptiX_LIBRARY}")


  
else()
  message(STATUS "Could not find OptiX")
endif()
