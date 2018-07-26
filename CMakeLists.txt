cmake_minimum_required(VERSION 2.8.3)
project(iiot_secure)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  genmsg
  rospy
)
# catkin_python_setup()

# Generate messages in the 'msg' folder
add_message_files(
  FILES
  Operation.msg
  OperationData.msg
)

# Generate services in the 'srv' folder
add_service_files(
  FILES
  ProcOperation.srv
  GetSensorsList.srv
  GetSensorData.srv
  SetSensorData.srv
  GetJournals.srv
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)
