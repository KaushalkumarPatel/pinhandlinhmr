cmake_minimum_required(VERSION 2.8.3)
project(yolov8_ros)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  yolov8_msgs
  message_generation
)

catkin_package()

install(DIRECTORY
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

