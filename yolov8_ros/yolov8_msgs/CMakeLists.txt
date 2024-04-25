cmake_minimum_required(VERSION 3.0.2)
project(yolov8_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  sensor_msgs
  geometry_msgs)

#include_directories(include)

add_message_files(
  DIRECTORY msg
  FILES
  Point2D.msg
  Vector2.msg
  Pose2D.msg
  BoundingBox2D.msg
  BoundingBox3D.msg
  Mask.msg
  KeyPoint2D.msg
  KeyPoint2DArray.msg
  KeyPoint3D.msg
  KeyPoint3DArray.msg
  Detection.msg
  DetectionArray.msg
)

generate_messages(DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
)

catkin_package(CATKIN_DEPENDS
  message_runtime
  std_msgs
  sensor_msgs
  geometry_msgs
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


#install(DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#  FILES_MATCHING PATTERN "*.h"
#)
#
#if (CATKIN_ENABLE_TESTING)
#  add_subdirectory(test)
#endif()