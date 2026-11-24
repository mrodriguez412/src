cmake_minimum_required(VERSION 3.8)
project(vrep4_helpers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -g -ggdb -O3)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(depth_image_proc REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)

add_executable(image_flip src/image_flip.cpp)
target_include_directories(image_flip PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(image_flip PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(image_flip PUBLIC
    ${rosgraph_msgs_TARGETS}
    ${sensor_msgs_TARGETS}
    ${std_msgs_TARGETS}
    cv_bridge::cv_bridge
    image_transport::image_transport
    rclcpp::rclcpp
    sensor_msgs::sensor_msgs_library
)


install(TARGETS image_flip
  DESTINATION lib/${PROJECT_NAME})

add_executable(clock_repeat src/clock_repeat.cpp)
target_include_directories(clock_repeat PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(clock_repeat PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(clock_repeat PUBLIC
    ${rosgraph_msgs_TARGETS}
    ${sensor_msgs_TARGETS}
    ${std_msgs_TARGETS}
    cv_bridge::cv_bridge
    image_transport::image_transport
    rclcpp::rclcpp
    sensor_msgs::sensor_msgs_library
)


install(TARGETS clock_repeat
  DESTINATION lib/${PROJECT_NAME})

install(
    DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
