cmake_minimum_required(VERSION 3.5)
project(coco_services)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/LaunchService.srv"
  DEPENDENCIES std_msgs std_srvs
)

install(PROGRAMS
  launch/launch_master_service
  launch/launch_listener
  
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
