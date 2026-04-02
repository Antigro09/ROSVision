cmake_minimum_required(VERSION 3.16)
project(ros_vision VERSION 0.1.0 LANGUAGES CXX)

# ---------------------------------------------------------------------------
# C++ standard
# ---------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ---------------------------------------------------------------------------
# Find ROS 2 / ament dependencies
# ---------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

# ---------------------------------------------------------------------------
# NVIDIA Isaac ROS AprilTag — REQUIRED (CUDA-accelerated detection backend)
# Installation:
#   Follow the Isaac ROS AprilTag guide at
#   https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_apriltag/
#   Requires: Jetson Orin / x86-64 with CUDA, ROS 2 Humble/Iron,
#             isaac_ros_common workspace set up.
# ---------------------------------------------------------------------------
find_package(isaac_ros_apriltag_interfaces REQUIRED)

# ---------------------------------------------------------------------------
# System dependencies
# ---------------------------------------------------------------------------
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(OpenCV REQUIRED)

# nlohmann/json — fetch if not available system-wide
find_package(nlohmann_json QUIET)
if(NOT nlohmann_json_FOUND)
  include(FetchContent)
  # Download the single-header release directly.
  FetchContent_Declare(
    nlohmann_json_dl
    URL      https://github.com/nlohmann/json/releases/download/v3.11.3/json.hpp
    URL_HASH SHA256=9bea4c8066ef4a1c206b2be5a36302f8926f7fdc6087af5d20b417d0cf103ea6
    DOWNLOAD_NO_EXTRACT TRUE
    DOWNLOAD_DIR        "${CMAKE_BINARY_DIR}/_json_include/nlohmann"
  )
  FetchContent_MakeAvailable(nlohmann_json_dl)
  add_library(ros_vision_nlohmann_json INTERFACE)
  target_include_directories(ros_vision_nlohmann_json INTERFACE
    "${CMAKE_BINARY_DIR}/_json_include")
  add_library(nlohmann_json::nlohmann_json ALIAS ros_vision_nlohmann_json)
  message(STATUS "nlohmann/json v3.11.3 fetched via FetchContent")
endif()

# ---------------------------------------------------------------------------
# WPILib ntcore (NetworkTables 4) — optional but strongly recommended.
#
# Install via the WPILib installer or the unofficial Debian packages:
#   https://github.com/wpilibsuite/allwpilib/releases
#
# On aarch64 (Jetson) you can build ntcore from source:
#   git clone https://github.com/wpilibsuite/allwpilib && cd allwpilib
#   cmake -B build -DWITH_JAVA=OFF -DWITH_CSCORE=OFF -DWITH_NTCORE=ON \
#         -DWITH_WPIMATH=OFF -DWITH_GUI=OFF -DWITH_SIMULATION_MODULES=OFF
#   cmake --build build --target ntcore -j$(nproc)
#   sudo cmake --install build
#
# If ntcore is not found, the NT publisher falls back to a console-log stub
# so the rest of the pipeline still builds and runs.
# ---------------------------------------------------------------------------
find_package(ntcore QUIET)
if(NOT ntcore_FOUND)
  find_package(PkgConfig QUIET)
  if(PkgConfig_FOUND)
    pkg_check_modules(NTCORE QUIET ntcore)
  endif()
  if(NTCORE_FOUND)
    add_library(ntcore::ntcore INTERFACE IMPORTED)
    target_include_directories(ntcore::ntcore INTERFACE ${NTCORE_INCLUDE_DIRS})
    target_link_libraries(ntcore::ntcore INTERFACE ${NTCORE_LINK_LIBRARIES})
    set(ntcore_FOUND TRUE)
  else()
    message(STATUS
      "ntcore not found — NT publisher will use a console-log stub. "
      "See README.md § 'Installing ntcore' for build instructions.")
  endif()
endif()

# ---------------------------------------------------------------------------
# Source files
# ---------------------------------------------------------------------------
set(VISION_SOURCES
  src/coordinate_transforms.cpp
  src/field_layout.cpp
  src/pose_estimation.cpp
  src/nt_publisher.cpp
  src/vision_node.cpp
)

add_executable(vision_node ${VISION_SOURCES})

target_include_directories(vision_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Ament dependencies (all required)
ament_target_dependencies(vision_node
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  cv_bridge
  isaac_ros_apriltag_interfaces
)

# Other libraries
target_link_libraries(vision_node
  Eigen3::Eigen
  ${OpenCV_LIBS}
  nlohmann_json::nlohmann_json
)
if(ntcore_FOUND)
  target_link_libraries(vision_node ntcore::ntcore)
  target_compile_definitions(vision_node PRIVATE HAVE_NTCORE)
endif()

# ---------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------
install(TARGETS vision_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/camera_calibration_gui
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
