cmake_minimum_required(VERSION 2.8.3)
project(godel_irb2400_support)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY
  config
  launch
  rviz
  urdf
  yaml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
