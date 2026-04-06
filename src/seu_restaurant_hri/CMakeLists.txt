cmake_minimum_required(VERSION 3.0.2)
project(seu_restaurant_hri)

find_package(catkin REQUIRED COMPONENTS
  rospy
  seu_restaurant_common
  seu_restaurant_msgs
  seu_speech_recognition
  std_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS rospy seu_restaurant_common seu_restaurant_msgs seu_speech_recognition std_msgs
)

include_directories(${catkin_INCLUDE_DIRS})

install(PROGRAMS
  scripts/mock_hri_node.py
  scripts/real_hri_node.py
  scripts/voice_order_client.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY src/seu_restaurant_hri/
  DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
)
install(DIRECTORY config/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
