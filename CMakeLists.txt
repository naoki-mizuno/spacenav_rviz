cmake_minimum_required(VERSION 2.8.3)
project(spacenav_rviz)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package()

catkin_install_python(
  PROGRAMS
    nodes/spacenav_tf.py
  DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY
    launch/
  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(
  DIRECTORY
    rviz/
  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
)

install(
  DIRECTORY
    config/
  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
