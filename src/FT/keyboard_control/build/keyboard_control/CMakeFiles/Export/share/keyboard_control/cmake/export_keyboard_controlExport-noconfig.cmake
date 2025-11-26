#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "keyboard_control::keyboard_control" for configuration ""
set_property(TARGET keyboard_control::keyboard_control APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(keyboard_control::keyboard_control PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/keyboard_control/keyboard_control"
  )

list(APPEND _IMPORT_CHECK_TARGETS keyboard_control::keyboard_control )
list(APPEND _IMPORT_CHECK_FILES_FOR_keyboard_control::keyboard_control "${_IMPORT_PREFIX}/lib/keyboard_control/keyboard_control" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
