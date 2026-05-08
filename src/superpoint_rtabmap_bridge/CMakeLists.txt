cmake_minimum_required(VERSION 3.8)
project(superpoint_rtabmap_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rtabmap_msgs REQUIRED)
find_package(rtabmap_conversions REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(OpenCV REQUIRED)

set(SP_RTABMAP_TORCH_PREFIX "" CACHE PATH "Path to the libtorch root directory")
set(SP_RTABMAP_EXTRA_RPATH "" CACHE STRING "Extra runtime library path, for example libtorch/lib")

set(SP_RTABMAP_DEFAULT_TORCH_PREFIX "/home/xu/project/SuperPointSLAM3-main_internet/third_party/libtorch-cpu-2.11.0/libtorch")

if(NOT SP_RTABMAP_TORCH_PREFIX AND EXISTS "${SP_RTABMAP_DEFAULT_TORCH_PREFIX}/share/cmake/Torch/TorchConfig.cmake")
  set(SP_RTABMAP_TORCH_PREFIX "${SP_RTABMAP_DEFAULT_TORCH_PREFIX}")
endif()

if(SP_RTABMAP_TORCH_PREFIX)
  list(APPEND CMAKE_PREFIX_PATH "${SP_RTABMAP_TORCH_PREFIX}")
endif()

find_package(Torch REQUIRED)

find_path(RTABMAP_CORE_INCLUDE_DIR
  NAMES rtabmap/core/Compression.h
  PATHS /opt/ros/humble/include/rtabmap-0.22
        /usr/include/rtabmap-0.22
  REQUIRED
)

find_library(RTABMAP_CORE_LIBRARY
  NAMES rtabmap_core
  PATHS /opt/ros/humble/lib/x86_64-linux-gnu /usr/lib/x86_64-linux-gnu
  REQUIRED
)

separate_arguments(TORCH_CXX_FLAGS_LIST NATIVE_COMMAND "${TORCH_CXX_FLAGS}")

add_executable(superpoint_rtabmap_bridge_node
  src/superpoint_rtabmap_bridge_node.cpp
  third_party/superpoint/SuperPointExtractor.cc
)

target_compile_options(superpoint_rtabmap_bridge_node PRIVATE ${TORCH_CXX_FLAGS_LIST})

target_include_directories(superpoint_rtabmap_bridge_node PRIVATE
  include
  third_party/superpoint
  ${OpenCV_INCLUDE_DIRS}
  ${RTABMAP_CORE_INCLUDE_DIR}
)

ament_target_dependencies(superpoint_rtabmap_bridge_node
  rclcpp
  sensor_msgs
  rtabmap_msgs
  rtabmap_conversions
  cv_bridge
  message_filters
)

target_link_libraries(superpoint_rtabmap_bridge_node
  ${OpenCV_LIBS}
  ${TORCH_LIBRARIES}
  ${RTABMAP_CORE_LIBRARY}
)

set_target_properties(superpoint_rtabmap_bridge_node PROPERTIES
  BUILD_RPATH "${SP_RTABMAP_EXTRA_RPATH}"
  INSTALL_RPATH "$ORIGIN;${SP_RTABMAP_EXTRA_RPATH}"
)

install(TARGETS superpoint_rtabmap_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
