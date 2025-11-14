cmake_minimum_required(VERSION 3.8)
project(mecanumbot_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

###############################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

################################################################################
# Declare ROS messages, services and actions
################################################################################
set(msg_files
  "msg/SensorState.msg"
<<<<<<< HEAD
<<<<<<< HEAD
  "msg/AccessoryMotors.msg"
=======
  "msg/AccessMotorCmd.msg"
  "msg/OpenCRState.msg"
>>>>>>> 1bffb20 (Create new message types for handling opencr memory and position commands - Csenge)
=======
  "msg/AccessMotorCmd.msg"
  "msg/OpenCRState.msg"
>>>>>>> 144f6513fe58770f2edaa42a55d4d407e47df7e0
)

set(srv_files
  "srv/GetLedStatus.srv"
  "srv/SetLedStatus.srv"
)

#set(action_files
#  
#)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  #${action_files}
  DEPENDENCIES std_msgs geometry_msgs #action_msgs
  ADD_LINTER_TESTS
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

ament_export_dependencies(rosidl_default_runtime)
ament_package()
