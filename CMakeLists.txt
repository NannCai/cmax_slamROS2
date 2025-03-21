cmake_minimum_required(VERSION 3.5)

project(cmax_slam)

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(OpenCV REQUIRED)
find_package(image_transport REQUIRED)
find_package(event_camera_codecs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(image_geometry REQUIRED)
find_package(glog REQUIRED)
find_package(GSL REQUIRED)

# find_package(rqt_image_view REQUIRED)  

set(CMAKE_CXX_STANDARD 14)
# set(CMAKE_CXX_STANDARD 17)
# if(NOT CMAKE_CXX_STANDARD)
#   set(CMAKE_CXX_STANDARD 17)
# endif()
# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()
# add_compile_options(-O3)


# add_compile_options(-Wno-class-memaccess -Wno-ignored-attributes)


file(GLOB SOURCES src/frontend/*.cpp
                  src/backend/*.cpp
                  src/utils/*.cpp
                  src/*.cpp)

# set(SOURCES
#   src/cmax_slam.cpp
#   src/node.cpp
# )

add_executable(${PROJECT_NAME} ${SOURCES})


target_include_directories(cmax_slam PRIVATE
  include
  ${EIGEN3_INCLUDE_DIRS}
  thirdparty/basalt-headers/thirdparty/Sophus
  thirdparty/basalt-headers/thirdparty/cereal/include
  thirdparty/basalt-headers/include
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_components
  OpenCV
  event_camera_msgs
  event_camera_codecs
  sensor_msgs
  image_transport  
  cv_bridge
  image_geometry
  camera_info_manager
  GSL
  geometry_msgs
)

target_link_libraries(cmax_slam
  glog::glog
)

# 安装规则
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY 
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

# 导出依赖
ament_export_dependencies(
  rclcpp
  OpenCV
)

ament_package()
