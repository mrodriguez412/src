cmake_minimum_required(VERSION 3.8)
project(floor_plane_regression_base)
cmake_policy(SET CMP0074 NEW)
set(CMAKE_POLICY_DEFAULT_CMP0144 NEW)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -g -ggdb -O3)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cs7630_msgs QUIET)
find_package(PCL REQUIRED QUIET COMPONENTS common io)
include_directories(${PCL_COMMON_INCLUDE_DIRS})

if( ${cs7630_msgs_FOUND} )
    set(cs7630_msgs_opt_TARGETS ${cs7630_msgs_TARGETS})
    add_compile_options(-DWITH_CS7630_MSGS)
    message("CS7630_msgs has been found.")
else()
    message("CS7630_msgs was not found. You may need to include it in your workspace.")
endif()


add_executable(floor_plane_regression src/floor_plane_regression.cpp)
target_include_directories(floor_plane_regression PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(floor_plane_regression PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(floor_plane_regression PUBLIC
    ${geometry_msgs_TARGETS}
    ${sensor_msgs_TARGETS}
    ${std_msgs_TARGETS}
    ${visualization_msgs_TARGETS}
    ${cs7630_msgs_opt_TARGETS}
    pcl_conversions::pcl_conversions
    sensor_msgs::sensor_msgs_library
    tf2_geometry_msgs::tf2_geometry_msgs
    tf2_ros::static_transform_broadcaster_node
    tf2_ros::tf2_ros
    tf2_sensor_msgs::tf2_sensor_msgs
    ${PCL_LIBRARIES}
)


install(TARGETS floor_plane_regression
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
