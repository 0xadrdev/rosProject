# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_paddle_physics_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED paddle_physics_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(paddle_physics_FOUND FALSE)
  elseif(NOT paddle_physics_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(paddle_physics_FOUND FALSE)
  endif()
  return()
endif()
set(_paddle_physics_CONFIG_INCLUDED TRUE)

# output package information
if(NOT paddle_physics_FIND_QUIETLY)
  message(STATUS "Found paddle_physics: 0.0.0 (${paddle_physics_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'paddle_physics' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${paddle_physics_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(paddle_physics_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${paddle_physics_DIR}/${_extra}")
endforeach()
