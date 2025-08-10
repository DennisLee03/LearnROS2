cmake_minimum_required(VERSION 3.8)
project(demo_service)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)

# generate custom interfaces
rosidl_generate_interfaces("${PROJECT_NAME}"
  srv/Vector.srv
  ADD_LINTER_TESTS
)

# set support for using custom interfaces in C++ for this package:
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

add_executable(vec_server src/vec_server.cpp)
ament_target_dependencies(vec_server rclcpp)
target_link_libraries(vec_server ${cpp_typesupport_target})

add_executable(vec_client src/vec_client.cpp)
ament_target_dependencies(vec_client rclcpp)
target_link_libraries(vec_client ${cpp_typesupport_target})

install(TARGETS
  vec_server
  vec_client
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
