cmake_minimum_required(VERSION 3.0.2)
project(seu_restaurant_manipulation)

find_package(catkin REQUIRED COMPONENTS
  rospy
  seu_restaurant_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy seu_restaurant_msgs
)

include_directories(${catkin_INCLUDE_DIRS})

install(PROGRAMS scripts/mock_manipulation_node.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY config/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
