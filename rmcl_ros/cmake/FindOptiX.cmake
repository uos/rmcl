### FindOptiX
# Variables Set:
# Pure Optix:
# - OptiX_INCLUDE_DIR
# - OptiX_LIBRARY
# - OptiX_VERSION
# - OptiX_FOUND
# Additional:
# - OptiX_INCLUDE_DIRS
# - Optix_LIBRARIES
# Aliases:
# example: OPTIX_INCLUDE_DIR = OptiX_INCLUDE_DIR



find_package(PkgConfig)
pkg_check_modules(OptiX QUIET optix)

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

# LIBRARY

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

if(NOT OptiX_LIBRARY)
  message(STATUS "OptiX library not found")
endif()
if(NOT OptiX_INCLUDE_DIR)
  message(STATUS "OptiX headers not found")
endif()

if(OptiX_LIBRARY AND OptiX_INCLUDE_DIR)
  set(OptiX_FOUND TRUE)
else()
  message(STATUS "Could not find OptiX")
endif()

message(STATUS "Include: ${OptiX_INCLUDE_DIR}")

if(OptiX_FOUND)
  set(OptiX_LIBRARIES ${OptiX_LIBRARY})
  set(OptiX_INCLUDE_DIRS ${OptiX_INCLUDE_DIR})
endif()



# aliases
set(OPTIX_FOUND ${OptiX_FOUND})
set(OPTIX_LIBRARY ${OptiX_LIBRARY})
set(OPTIX_INCLUDE_DIR ${OptiX_INCLUDE_DIR})
set(OPTIX_INCLUDE_DIRS ${OptiX_INCLUDE_DIRS})
set(OPTIX_VERSION ${OptiX_VERSION})
set(OPTIX_LIBRARIES ${OptiX_LIBRARIES})
