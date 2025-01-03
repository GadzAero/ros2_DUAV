#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "joy::game_controller" for configuration ""
set_property(TARGET joy::game_controller APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(joy::game_controller PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libgame_controller.so"
  IMPORTED_SONAME_NOCONFIG "libgame_controller.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS joy::game_controller )
list(APPEND _IMPORT_CHECK_FILES_FOR_joy::game_controller "${_IMPORT_PREFIX}/lib/libgame_controller.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
