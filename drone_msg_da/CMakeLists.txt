cmake_minimum_required(VERSION 3.22)
project(drone_msg_da)

set(CMAKE_CXX_STANDARD 23)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/Battery.msg
  msg/CameraState.msg
  msg/CurrentPosition.msg
  msg/Failures.msg
  msg/FlightModes.msg
  msg/GimbalAdjustmentDto.msg
  msg/GimbalState.msg
  msg/Goto.msg
  msg/Gps.msg
  msg/HardwareStatus.msg
  msg/MavlinkPayload.msg
  msg/Misc.msg
  msg/Movement.msg
  msg/Position.msg
  msg/Geofence.msg
  msg/Elp.msg
  msg/ReceivedMission.msg
  msg/SoftwareStatus.msg
  msg/CompanionStatus.msg
  msg/DroneState.msg
  msg/Status.msg
  msg/Task.msg
  msg/Velocity.msg
  msg/Waypoint.msg
  msg/Mission.msg
  msg/LinearGrabberState.msg
  msg/Target.msg
  msg/StringMap.msg

  srv/CaptureInterval.srv
  srv/CaptureMode.srv
  srv/CapturePhoto.srv
  srv/CaptureVideoStart.srv
  srv/CaptureVideoStop.srv
  srv/ConfigRtk.srv
  srv/DisplayMode.srv
  srv/GimbalAdjustment.srv
  srv/Goto.srv
  srv/Mission.srv
  srv/Push.srv
  srv/ReqConfig.srv
  srv/SendMission.srv
  srv/StartMission.srv
  srv/TapAndFlyWaypoint.srv
  srv/TriggerAutoFocus.srv
  srv/TriggerBracket.srv
  DEPENDENCIES builtin_interfaces std_msgs
)

ament_export_dependencies(rosidl_default_runtime builtin_interfaces std_msgs)

ament_package()
