cmake_minimum_required(VERSION 3.16)
project(bof_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ArmorCandidate.msg" "msg/ArmorCandidates.msg" "msg/PixelTrack.msg"
  "msg/ArmorPose.msg" "msg/ArmorPoses.msg"
  "msg/SpatialTarget.msg" "msg/AimSolution.msg"
  "msg/AimDirective.msg" "msg/AutoAimTarget.msg" "msg/GimbalState.msg" "msg/GimbalCommand.msg"
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
