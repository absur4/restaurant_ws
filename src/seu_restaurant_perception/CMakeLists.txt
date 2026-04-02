cmake_minimum_required(VERSION 3.0.2)
project(seu_restaurant_perception)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  sensor_msgs
  std_msgs
  tf2_geometry_msgs
  tf2_ros
  rospy
  seu_restaurant_msgs
)

catkin_package(
  CATKIN_DEPENDS cv_bridge geometry_msgs rospy sensor_msgs seu_restaurant_msgs std_msgs tf2_geometry_msgs tf2_ros
)

include_directories(${catkin_INCLUDE_DIRS})

install(PROGRAMS
  scripts/mock_perception_node.py
  scripts/mediapipe_wave_detector.py
  scripts/wave_to_customer_call.py
  scripts/perception_visualizer.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY config/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
