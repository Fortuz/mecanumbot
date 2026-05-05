cmake_minimum_required(VERSION 3.8)
project(mecanumbot_cam_optim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(CURL REQUIRED)

add_executable(camera_stream_node src/camera_stream.cpp)
target_include_directories(camera_stream_node PUBLIC ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(camera_stream_node rclcpp sensor_msgs cv_bridge image_transport)
target_link_libraries(camera_stream_node ${OpenCV_LIBRARIES} ${CURL_LIBRARIES})

install(TARGETS
  camera_stream_node
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
