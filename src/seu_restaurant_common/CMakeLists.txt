cmake_minimum_required(VERSION 3.0.2)
project(seu_restaurant_common)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  seu_restaurant_msgs
  std_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS geometry_msgs rospy seu_restaurant_msgs std_msgs
)

include_directories(${catkin_INCLUDE_DIRS})

install(DIRECTORY src/seu_restaurant_common/
  DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
