cmake_minimum_required(VERSION 3.8)
project(drone_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# Генерируем ROS2-типы из .msg. Target.msg ссылается на std_msgs (Header),
# поэтому std_msgs объявлен зависимостью генерации.
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Target.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
