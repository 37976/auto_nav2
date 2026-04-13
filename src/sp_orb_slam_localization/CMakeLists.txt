cmake_minimum_required(VERSION 3.8)
project(sp_orb_slam_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

set(SP_ORB_SLAM_TORCH_PREFIX "" CACHE PATH "Path to the libtorch root directory")
set(SP_ORB_SLAM_EXTRA_RPATH "" CACHE STRING "Extra runtime library path, for example libtorch/lib")

if(SP_ORB_SLAM_TORCH_PREFIX)
  list(APPEND CMAKE_PREFIX_PATH "${SP_ORB_SLAM_TORCH_PREFIX}")
endif()

find_package(Torch REQUIRED)

set(SP_ORB_SLAM_VENDOR_DIR "${CMAKE_CURRENT_SOURCE_DIR}/vendor/superpoint_orb_slam3")
set(SP_ORB_SLAM_MODEL_PATH "${SP_ORB_SLAM_VENDOR_DIR}/weights/superpoint.ts")

set(SP_ORB_SLAM_CORE_SOURCES
  ${SP_ORB_SLAM_VENDOR_DIR}/src/System.cc
  ${SP_ORB_SLAM_VENDOR_DIR}/src/Tracking.cc
  ${SP_ORB_SLAM_VENDOR_DIR}/src/LocalMapping.cc
  ${SP_ORB_SLAM_VENDOR_DIR}/src/ORBextractor.cc
  ${SP_ORB_SLAM_VENDOR_DIR}/src/ORBmatcher.cc
  ${SP_ORB_SLAM_VENDOR_DIR}/src/FrameDrawer.cc
  ${SP_ORB_SLAM_VENDOR_DIR}/src/Converter.cc
  ${SP_ORB_SLAM_VENDOR_DIR}/src/MapPoint.cc
  ${SP_ORB_SLAM_VENDOR_DIR}/src/KeyFrame.cc
  ${SP_ORB_SLAM_VENDOR_DIR}/src/Atlas.cc
  ${SP_ORB_SLAM_VENDOR_DIR}/src/Map.cc
  ${SP_ORB_SLAM_VENDOR_DIR}/src/Optimizer.cc
  ${SP_ORB_SLAM_VENDOR_DIR}/src/Frame.cc
  ${SP_ORB_SLAM_VENDOR_DIR}/src/Sim3Solver.cc
  ${SP_ORB_SLAM_VENDOR_DIR}/src/ImuTypes.cc
  ${SP_ORB_SLAM_VENDOR_DIR}/src/G2oTypes.cc
  ${SP_ORB_SLAM_VENDOR_DIR}/src/CameraModels/Pinhole.cpp
  ${SP_ORB_SLAM_VENDOR_DIR}/src/CameraModels/KannalaBrandt8.cpp
  ${SP_ORB_SLAM_VENDOR_DIR}/src/OptimizableTypes.cpp
  ${SP_ORB_SLAM_VENDOR_DIR}/src/MLPnPsolver.cpp
  ${SP_ORB_SLAM_VENDOR_DIR}/src/GeometricTools.cc
  ${SP_ORB_SLAM_VENDOR_DIR}/src/TwoViewReconstruction.cc
  ${SP_ORB_SLAM_VENDOR_DIR}/src/Config.cc
  ${SP_ORB_SLAM_VENDOR_DIR}/src/Settings.cc
  ${SP_ORB_SLAM_VENDOR_DIR}/src/SuperPointExtractor.cc
  ${SP_ORB_SLAM_VENDOR_DIR}/src/SuperPointMatcher.cc
)

set(SP_ORB_SLAM_PUBLIC_INCLUDE_DIRS
  ${SP_ORB_SLAM_VENDOR_DIR}
  ${SP_ORB_SLAM_VENDOR_DIR}/include
  ${SP_ORB_SLAM_VENDOR_DIR}/include/CameraModels
  ${SP_ORB_SLAM_VENDOR_DIR}/Thirdparty/Sophus
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)

set(SP_ORB_SLAM_THIRDPARTY_BUILD_RPATH
  ${SP_ORB_SLAM_VENDOR_DIR}/Thirdparty/DBoW2/lib
  ${SP_ORB_SLAM_VENDOR_DIR}/Thirdparty/g2o/lib
)

if(SP_ORB_SLAM_EXTRA_RPATH)
  list(APPEND SP_ORB_SLAM_THIRDPARTY_BUILD_RPATH ${SP_ORB_SLAM_EXTRA_RPATH})
endif()

add_subdirectory(${SP_ORB_SLAM_VENDOR_DIR}/Thirdparty/DBoW2 ${CMAKE_CURRENT_BINARY_DIR}/dbow2)
add_subdirectory(${SP_ORB_SLAM_VENDOR_DIR}/Thirdparty/g2o ${CMAKE_CURRENT_BINARY_DIR}/g2o)

separate_arguments(TORCH_CXX_FLAGS_LIST NATIVE_COMMAND "${TORCH_CXX_FLAGS}")

add_library(sp_orb_slam3_core SHARED ${SP_ORB_SLAM_CORE_SOURCES})
target_compile_options(sp_orb_slam3_core PRIVATE ${TORCH_CXX_FLAGS_LIST})
target_compile_definitions(sp_orb_slam3_core PRIVATE
  ORB_SLAM3_MINIMAL
  ORB_SLAM3_DEFAULT_SP_MODEL_PATH=\"${SP_ORB_SLAM_MODEL_PATH}\"
)
target_include_directories(sp_orb_slam3_core PUBLIC ${SP_ORB_SLAM_PUBLIC_INCLUDE_DIRS})
target_link_libraries(sp_orb_slam3_core
  PUBLIC
    Eigen3::Eigen
    ${OpenCV_LIBS}
    ${TORCH_LIBRARIES}
    DBoW2
    g2o
    boost_serialization
    crypto
)
set_target_properties(sp_orb_slam3_core PROPERTIES
  BUILD_RPATH "${SP_ORB_SLAM_THIRDPARTY_BUILD_RPATH}"
  INSTALL_RPATH "$ORIGIN;${SP_ORB_SLAM_EXTRA_RPATH}"
)

add_executable(superpoint_localization_node
  src/superpoint_localization_node.cpp
)
target_compile_options(superpoint_localization_node PRIVATE ${TORCH_CXX_FLAGS_LIST})
target_compile_definitions(superpoint_localization_node PRIVATE
  ORB_SLAM3_MINIMAL
)
target_include_directories(superpoint_localization_node PRIVATE ${SP_ORB_SLAM_PUBLIC_INCLUDE_DIRS})
ament_target_dependencies(superpoint_localization_node
  ament_index_cpp
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  cv_bridge
  tf2_ros
)
target_link_libraries(superpoint_localization_node
  sp_orb_slam3_core
  Eigen3::Eigen
  ${OpenCV_LIBS}
)
set_target_properties(superpoint_localization_node PROPERTIES
  BUILD_RPATH "${CMAKE_CURRENT_BINARY_DIR};${SP_ORB_SLAM_THIRDPARTY_BUILD_RPATH}"
  INSTALL_RPATH "$ORIGIN/..;${SP_ORB_SLAM_EXTRA_RPATH}"
)

install(TARGETS
  DBoW2
  g2o
  sp_orb_slam3_core
  DESTINATION lib
)

install(TARGETS superpoint_localization_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  ${SP_ORB_SLAM_VENDOR_DIR}/weights
  ${SP_ORB_SLAM_VENDOR_DIR}/Vocabulary
  DESTINATION share/${PROJECT_NAME}/vendor
)

ament_package()
