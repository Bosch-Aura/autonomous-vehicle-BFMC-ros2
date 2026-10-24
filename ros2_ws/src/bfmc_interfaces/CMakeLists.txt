cmake_minimum_required(VERSION 3.8)
project(bfmc_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LaneState.msg"
  "msg/SignDetection.msg"
  "msg/SignDetectionArray.msg"
  "msg/HardwareStatus.msg"
  "msg/VehicleState.msg"
  "action/ManeuverExecute.action"
  DEPENDENCIES std_msgs
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_interfaces test/test_interfaces.py)
endif()

ament_package()
