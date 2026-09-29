cmake_minimum_required(VERSION 3.8)
project(floor_nav)
cmake_policy(SET CMP0074 NEW)
set(CMAKE_POLICY_DEFAULT_CMP0144 NEW)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -std=c++17 -g -ggdb -O3)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclpy REQUIRED)
find_package(task_manager_msgs REQUIRED)
find_package(task_manager_lib REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(topic_tools_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cs7630_msgs REQUIRED)
find_package(kobuki_ros_interfaces REQUIRED)

find_package(PCL REQUIRED QUIET COMPONENTS common io)
find_package(pcl_conversions REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(task_manager_nav2)

include_directories(
  include
  ${PCL_COMMON_INCLUDE_DIRS}
  ${pcl_conversions_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${rclcpp_action_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${topic_tools_interfaces_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
  ${tf2_geometry_msgs_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${std_srvs_INCLUDE_DIRS}
  ${task_manager_msgs_INCLUDE_DIRS}
  ${task_manager_lib_INCLUDE_DIRS}
  ${task_manager_nav2_INCLUDE_DIRS}
  ${kobuki_ros_interfaces_INCLUDE_DIRS}
)

SET(ALL_LIBS 
    ${PCL_LIBRARIES} 
    ${pcl_conversions_LIBRARIES}
    ${rclcpp_LIBRARIES} 
    ${rclcpp_action_LIBRARIES} 
    ${task_manager_lib_LIBRARIES} 
    ${topic_tools_interfaces_LIBRARIES} 
    ${sensor_msgs_LIBRARIES} 
    ${geometry_msgs_LIBRARIES} 
    ${tf2_LIBRARIES} 
    ${tf2_geometry_msgs_LIBRARIES} 
    ${task_manager_msgs_LIBRARIES} 
    ${std_msgs_LIBRARIES} 
    ${std_srvs_LIBRARIES} 
    ${task_manager_nav2_LIBRARIES} 
    ${kobuki_ros_interfaces_LIBRARIES}
    ${Boost_THREAD_LIBRARIES} 
    ${CMAKE_DL_LIBS})

SET(ALL_DEPS task_manager_lib pcl_conversions
    task_manager_msgs geometry_msgs rclcpp rclcpp_action std_msgs task_manager_nav2
    tf2 tf2_geometry_msgs)

file(SHA256 ${PROJECT_SOURCE_DIR}/include/floor_nav/SimTasksEnv.h ENV_CHECK_SUM)
add_compile_definitions(ENV_CHECK_SUM=${ENV_CHECK_SUM})

ADD_EXECUTABLE( floornav_task_server src/task_server.cpp src/SimTasksEnv.cpp) 
target_link_libraries(floornav_task_server PUBLIC
	${geometry_msgs_TARGETS}
	${std_msgs_TARGETS}
	${cs7630_msgs_TARGETS}
	${task_manager_msgs_TARGETS}
	pcl_conversions::pcl_conversions
	rclcpp::rclcpp
	rclcpp_action::rclcpp_action
	tf2::tf2
	tf2_geometry_msgs::tf2_geometry_msgs
	${ALL_LIBS}
)
# Necessary target for the tasks to see the symbols of the main
TARGET_LINK_OPTIONS(floornav_task_server PRIVATE "-rdynamic")
# TARGET_LINK_LIBRARIES(floornav_task_server ${ALL_LIBS})

macro(add_task)
	message("Adding task ${ARGV0}")
	ADD_LIBRARY( floornav_${ARGV0} SHARED tasks/${ARGV0}.cpp)
	# ament_target_dependencies(floornav_${ARGV0} ${ALL_DEPS})
	target_link_libraries(floornav_${ARGV0} PUBLIC
		${geometry_msgs_TARGETS}
		${std_msgs_TARGETS}
        ${cs7630_msgs_TARGETS}
		${task_manager_msgs_TARGETS}
		pcl_conversions::pcl_conversions
		rclcpp::rclcpp
		rclcpp_action::rclcpp_action
		tf2::tf2
		tf2_geometry_msgs::tf2_geometry_msgs
		${ALL_LIBS}
	)
	install(TARGETS floornav_${ARGV0} DESTINATION lib/${PROJECT_NAME})
endmacro()

# Moved to generic task system
# add_task(TaskTrigger)
# add_task(TaskSetBool)
add_task(TaskNav2)
add_task(TaskSetMux)
add_task(TaskConstant)
add_task(TaskGoTo)
add_task(TaskSetHeading)
add_task(TaskPlanTo)
add_task(TaskWaitForROI)
add_task(TaskWander)
add_task(TaskAutoDock)

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

install(TARGETS
  floornav_task_server
  DESTINATION lib/${PROJECT_NAME})


install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}
)


install(PROGRAMS  
	missions/graph_forever.py
	missions/test_goto.py
	missions/test_graph_2d.py
	missions/test_graph.py
	missions/test_graph-test.py
	missions/test_planto.py
	missions/test_set_heading.py
	missions/test_wander.py
    DESTINATION lib/${PROJECT_NAME} 
)
ament_package()
