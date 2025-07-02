cmake_minimum_required(VERSION 3.0.2)
project(dynamic_obstacle_avoidance)

find_package(catkin REQUIRED COMPONENTS
  rospy
  duckietown_msgs
  sensor_msgs
  geometry_msgs
  std_msgs
  cv_bridge
  message_generation
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  dynamic_obstacle.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS 
    rospy 
    duckietown_msgs 
    sensor_msgs 
    geometry_msgs 
    std_msgs 
    cv_bridge
    message_runtime
)

catkin_install_python(
  PROGRAMS 
    src/dynamic_controller_node.py
    src/led_detection_node.py
    src/duckie_detection_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
) 