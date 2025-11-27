# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_screen_ctrl_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED screen_ctrl_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(screen_ctrl_FOUND FALSE)
  elseif(NOT screen_ctrl_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(screen_ctrl_FOUND FALSE)
  endif()
  return()
endif()
set(_screen_ctrl_CONFIG_INCLUDED TRUE)

# output package information
if(NOT screen_ctrl_FIND_QUIETLY)
  message(STATUS "Found screen_ctrl: 0.0.0 (${screen_ctrl_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'screen_ctrl' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${screen_ctrl_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(screen_ctrl_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${screen_ctrl_DIR}/${_extra}")
endforeach()
