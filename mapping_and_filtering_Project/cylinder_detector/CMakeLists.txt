cmake_minimum_required(VERSION 3.8)
project(cylinder_detector)
cmake_policy(SET CMP0074 NEW)
set(CMAKE_POLICY_DEFAULT_CMP0074 NEW)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -O3)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED QUIET COMPONENTS common io)

add_executable(cylinder_detector src/cylinder_detector.cpp)

ament_target_dependencies(cylinder_detector
  rclcpp
  sensor_msgs
  geometry_msgs
  visualization_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
  pcl_conversions
)
target_link_libraries(cylinder_detector
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  Eigen3::Eigen
)

install(TARGETS cylinder_detector
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
