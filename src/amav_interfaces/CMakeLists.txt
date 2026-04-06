cmake_minimum_required(VERSION 3.8)
project(amav_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BoundingBox2D.msg"
  "msg/Detection.msg"
  "msg/DetectionArray.msg"
  "msg/TrackedObject.msg"
  "msg/TrackedObjectArray.msg"
  "msg/AgentStatus.msg"
  "srv/Handoff.srv"
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs builtin_interfaces
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Export the interfaces
ament_export_dependencies(rosidl_default_runtime)

ament_package()
