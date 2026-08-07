# ROS 2 distro codenames increment alphabetically each release
# (Humble, Iron, Jazzy, Kilted, Lyrical, ...), so the first letter tells us how
# new the distro is. This computes ROS_DISTRO_NUMBER (A=1, B=2, ...) and defines
# it as a compile definition, so C++ code can do plain integer comparisons for
# API differences across distros, e.g. `#if ROS_DISTRO_NUMBER <= ROS_DISTRO_JAZZY`
# (see rmcl_ros/include/rmcl_ros/util/ros_helper.h for the named distro thresholds).
macro(rmcl_define_ros_distro_number)
  if(DEFINED ENV{ROS_DISTRO})
    string(SUBSTRING "$ENV{ROS_DISTRO}" 0 1 _rmcl_distro_letter)
    string(TOLOWER "${_rmcl_distro_letter}" _rmcl_distro_letter)
  else()
    set(_rmcl_distro_letter "z") # unknown/rolling: assume newest
  endif()
  string(FIND "abcdefghijklmnopqrstuvwxyz" "${_rmcl_distro_letter}" _rmcl_distro_index)
  math(EXPR ROS_DISTRO_NUMBER "${_rmcl_distro_index} + 1")
  message(STATUS "ROS_DISTRO_NUMBER: ${ROS_DISTRO_NUMBER} (from $ENV{ROS_DISTRO})")
  add_compile_definitions(ROS_DISTRO_NUMBER=${ROS_DISTRO_NUMBER})
  unset(_rmcl_distro_letter)
  unset(_rmcl_distro_index)
endmacro()
