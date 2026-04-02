cmake_minimum_required(VERSION 3.0.2)
project(seu_restaurant_navigation)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  geometry_msgs
  move_base_msgs
  rospy
  seu_restaurant_common
  seu_restaurant_msgs
  tf2_geometry_msgs
  tf2_ros
)

catkin_package(
  CATKIN_DEPENDS actionlib actionlib_msgs geometry_msgs move_base_msgs rospy seu_restaurant_common seu_restaurant_msgs tf2_geometry_msgs tf2_ros
)

include_directories(${catkin_INCLUDE_DIRS})

install(PROGRAMS
  scripts/mock_navigation_node.py
  scripts/real_navigation_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY config/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
