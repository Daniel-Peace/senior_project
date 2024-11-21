cmake_minimum_required(VERSION 2.8.3)
project(messages)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs sensor_msgs)

add_message_files(
  FILES
  Assigned_apriltag.msg
  Casualty_prediction.msg
  Command.msg
  Critical_report_response.msg
  Critical_report.msg
  Critical.msg
  Current_timer.msg
  Injury_report_response.msg
  Injury_report.msg
  Injury.msg
  LoopState.msg
  ModelPredictionStatus.msg
  ModelPredictionStatuses.msg
  Remaining_reports.msg
  Response_status.msg
  Status.msg
  Timer_state.msg
  Vitals_report_response.msg
  Vitals_report.msg
  Vitals.msg
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

catkin_package(
   CATKIN_DEPENDS message_runtime
)

include_directories(
)
