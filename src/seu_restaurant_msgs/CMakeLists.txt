cmake_minimum_required(VERSION 3.0.2)
project(seu_restaurant_msgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  std_msgs
)

add_message_files(
  FILES
  CustomerCall.msg
  OrderItem.msg
  OrderInfo.msg
  ServiceTask.msg
)

add_service_files(
  FILES
  ParseOrder.srv
  NavToPose.srv
  SpeakText.srv
  ListenText.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs message_runtime std_msgs
)

include_directories(${catkin_INCLUDE_DIRS})
