cmake_minimum_required(VERSION 3.0.2)
project(seu_restaurant_smach)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  seu_restaurant_common
  seu_restaurant_msgs
  smach
  smach_ros
  std_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS geometry_msgs rospy seu_restaurant_common seu_restaurant_msgs smach smach_ros std_msgs
)

include_directories(${catkin_INCLUDE_DIRS})

install(PROGRAMS
  scripts/restaurant_smach.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY src/seu_restaurant_smach/
  DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
