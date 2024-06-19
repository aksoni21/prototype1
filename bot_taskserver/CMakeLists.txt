cmake_minimum_required(VERSION 3.8)
project(bot_taskserver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(bot_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_library(main_taskserver SHARED src/main_taskserver.cpp)
target_include_directories(main_taskserver PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(main_taskserver
  PRIVATE "MAIN_TASKSERVER_CPP_BUILDING_DLL")

ament_target_dependencies(main_taskserver
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "moveit_ros_planning_interface"
  "bot_msgs" 
)

rclcpp_components_register_node(main_taskserver
  PLUGIN "bot_taskserver::TaskServer"
  EXECUTABLE main_taskserver_node
)

install(TARGETS
main_taskserver
ARCHIVE DESTINATION lib
LIBRARY DESTINATION lib
RUNTIME DESTINATION bin
)

add_executable(simple_taskserver src/simple_taskserver.cpp)
ament_target_dependencies(simple_taskserver rclcpp moveit_ros_planning_interface)

# add_executable(main_taskserver src/main_taskserver.cpp)
# ament_target_dependencies(main_taskserver rclcpp moveit_ros_planning_interface)


install(
TARGETS simple_taskserver 
DESTINATION lib/${PROJECT_NAME}
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
