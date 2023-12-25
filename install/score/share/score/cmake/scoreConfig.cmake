# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_score_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED score_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(score_FOUND FALSE)
  elseif(NOT score_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(score_FOUND FALSE)
  endif()
  return()
endif()
set(_score_CONFIG_INCLUDED TRUE)

# output package information
if(NOT score_FIND_QUIETLY)
  message(STATUS "Found score: 0.0.0 (${score_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'score' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${score_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(score_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${score_DIR}/${_extra}")
endforeach()
