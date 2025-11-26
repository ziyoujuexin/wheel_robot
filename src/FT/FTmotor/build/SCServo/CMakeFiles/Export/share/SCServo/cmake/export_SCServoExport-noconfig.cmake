#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "SCServo::SCServo" for configuration ""
set_property(TARGET SCServo::SCServo APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(SCServo::SCServo PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libSCServo.so"
  IMPORTED_SONAME_NOCONFIG "libSCServo.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS SCServo::SCServo )
list(APPEND _IMPORT_CHECK_FILES_FOR_SCServo::SCServo "${_IMPORT_PREFIX}/lib/libSCServo.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
