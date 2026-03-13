#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "dynamicvoronoi::dynamicvoronoi" for configuration ""
set_property(TARGET dynamicvoronoi::dynamicvoronoi APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(dynamicvoronoi::dynamicvoronoi PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libdynamicvoronoi.so"
  IMPORTED_SONAME_NOCONFIG "libdynamicvoronoi.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS dynamicvoronoi::dynamicvoronoi )
list(APPEND _IMPORT_CHECK_FILES_FOR_dynamicvoronoi::dynamicvoronoi "${_IMPORT_PREFIX}/lib/libdynamicvoronoi.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
