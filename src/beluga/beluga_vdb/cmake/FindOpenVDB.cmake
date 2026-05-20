find_path(
  OpenVDB_INCLUDE_DIR
  NAMES openvdb/openvdb.h
  PATH_SUFFIXES include)

find_library(
  OpenVDB_LIBRARY
  NAMES openvdb
  PATH_SUFFIXES lib lib64 lib/x86_64-linux-gnu)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(
  OpenVDB
  REQUIRED_VARS OpenVDB_INCLUDE_DIR OpenVDB_LIBRARY)

if(OpenVDB_FOUND AND NOT TARGET OpenVDB::openvdb)
  add_library(OpenVDB::openvdb UNKNOWN IMPORTED)
  set_target_properties(
    OpenVDB::openvdb
    PROPERTIES IMPORTED_LOCATION "${OpenVDB_LIBRARY}"
               INTERFACE_INCLUDE_DIRECTORIES "${OpenVDB_INCLUDE_DIR}")
endif()

mark_as_advanced(OpenVDB_INCLUDE_DIR OpenVDB_LIBRARY)
