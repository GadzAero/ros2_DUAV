#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "joy::joy" for configuration ""
set_property(TARGET joy::joy APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(joy::joy PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libjoy.so"
  IMPORTED_SONAME_NOCONFIG "libjoy.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS joy::joy )
list(APPEND _IMPORT_CHECK_FILES_FOR_joy::joy "${_IMPORT_PREFIX}/lib/libjoy.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
