cmake_minimum_required(VERSION 2.8.3)

project(yak_meshing)
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  rosconsole
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  tf
  pcl_ros
  visualization_msgs
  yak
  pcl_ros
  pcl_msgs
)

find_library(OPENVDB_LIBRARY NAMES openvdb)
find_library(TBB_LIBRARY NAMES tbb)
find_library(HALF_LIBRARY NAMES Half)

add_service_files(
 FILES
  GetMesh.srv
)

generate_messages(
 DEPENDENCIES
   std_msgs
   geometry_msgs
   pcl_msgs
 )
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    message_runtime
    rosconsole
    roscpp
    rospy
    std_msgs
    sensor_msgs
    tf
    pcl_ros
    pcl_msgs
    visualization_msgs
    yak
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/yak_meshing_node.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
 ${catkin_LIBRARIES}
 openvdb tbb Half
)
