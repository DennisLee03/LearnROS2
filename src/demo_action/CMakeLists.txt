cmake_minimum_required(VERSION 3.8)
project(demo_action)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)

# generate custom interfaces
rosidl_generate_interfaces("${PROJECT_NAME}"
  "action/Fibonacci.action"
  DEPENDENCIES 
  action_msgs
  ADD_LINTER_TESTS
)

rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

add_executable(fib_action_server src/fib_action_server.cpp)
ament_target_dependencies(fib_action_server rclcpp rclcpp_action)
target_link_libraries(fib_action_server "${cpp_typesupport_target}")

add_executable(fib_action_client src/fib_action_client.cpp)
ament_target_dependencies(fib_action_client rclcpp rclcpp_action)
target_link_libraries(fib_action_client "${cpp_typesupport_target}")

install(TARGETS
  fib_action_server
  fib_action_client
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
