cmake_minimum_required(VERSION 3.5)
project(cmax_slam)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(event_camera_codecs REQUIRED)
find_package(image_geometry REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

# Include directories
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${event_camera_codecs_INCLUDE_DIRS}
  ${image_geometry_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}

)

include_directories(thirdparty/basalt-headers/thirdparty/Sophus)
include_directories(thirdparty/basalt-headers/thirdparty/cereal/include)
include_directories(thirdparty/basalt-headers/include)

# Source files
set(SOURCES
  src/cmax_slam.cpp
  src/node.cpp
)

# Executable
add_executable(${PROJECT_NAME} ${SOURCES})

# Link libraries
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  sensor_msgs
  event_camera_codecs
  image_geometry
  cv_bridge
  OpenCV
)

# Install
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()