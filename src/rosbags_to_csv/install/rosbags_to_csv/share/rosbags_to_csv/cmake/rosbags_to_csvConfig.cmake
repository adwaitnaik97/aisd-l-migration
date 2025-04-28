# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_rosbags_to_csv_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED rosbags_to_csv_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(rosbags_to_csv_FOUND FALSE)
  elseif(NOT rosbags_to_csv_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(rosbags_to_csv_FOUND FALSE)
  endif()
  return()
endif()
set(_rosbags_to_csv_CONFIG_INCLUDED TRUE)

# output package information
if(NOT rosbags_to_csv_FIND_QUIETLY)
  message(STATUS "Found rosbags_to_csv: 0.0.0 (${rosbags_to_csv_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'rosbags_to_csv' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${rosbags_to_csv_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(rosbags_to_csv_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${rosbags_to_csv_DIR}/${_extra}")
endforeach()
