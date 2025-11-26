# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_SCServo_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED SCServo_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(SCServo_FOUND FALSE)
  elseif(NOT SCServo_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(SCServo_FOUND FALSE)
  endif()
  return()
endif()
set(_SCServo_CONFIG_INCLUDED TRUE)

# output package information
if(NOT SCServo_FIND_QUIETLY)
  message(STATUS "Found SCServo: 0.0.0 (${SCServo_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'SCServo' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${SCServo_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(SCServo_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "ament_cmake_export_include_directories-extras.cmake;ament_cmake_export_libraries-extras.cmake;ament_cmake_export_targets-extras.cmake")
foreach(_extra ${_extras})
  include("${SCServo_DIR}/${_extra}")
endforeach()
