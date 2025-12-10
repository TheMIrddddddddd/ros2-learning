cmake_minimum_required(VERSION 3.8)
project(demo_cpp_service)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(chapt4_interface REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(cv_bridge REQUIRED)

add_executable(demo_cpp_service src/demo_cpp_service.cpp)
add_executable(face_detect_server src/face_detect_server.cpp)
add_executable(face_detect_client src/face_detect_client.cpp)

ament_target_dependencies(face_detect_client
  rclcpp
  OpenCV
  sensor_msgs
  chapt4_interface
  ament_index_cpp
  cv_bridge
)

ament_target_dependencies(face_detect_server
  rclcpp
  OpenCV
  sensor_msgs
  chapt4_interface
  ament_index_cpp
  cv_bridge
)

ament_target_dependencies(demo_cpp_service
  rclcpp
  OpenCV
  sensor_msgs
  chapt4_interface
  ament_index_cpp
  cv_bridge
)
install(TARGETS demo_cpp_service face_detect_server face_detect_client
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY resource/
  DESTINATION share/${PROJECT_NAME}/resource
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
