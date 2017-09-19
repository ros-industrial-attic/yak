cmake_minimum_required(VERSION 2.8.3)
project(nbv_planner)
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  geometry_msgs
  roscpp
  rospy
  std_msgs
  octomap_ros
  pcl_ros
  tf
  moveit_ros_planning_interface
)

find_package(octomap REQUIRED)

add_service_files(
  FILES
  GetNBV.srv
)

generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
)

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    message_runtime
    geometry_msgs
    roscpp
    rospy
    std_msgs
    octomap_ros
    pcl_ros
    tf
    moveit_ros_planning_interface
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS})

link_libraries(${OCTOMAP_LIBRARIES})

add_executable(${PROJECT_NAME}_node src/nbv_planner_node.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_executable(octomap_reset_node src/octomap_reset_node.cpp)
add_dependencies(octomap_reset_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_executable(exploration_controller_node src/exploration_controller_node.cpp)
add_dependencies(exploration_controller_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


target_link_libraries(${PROJECT_NAME}_node
    ${catkin_LIBRARIES}
)

target_link_libraries(octomap_reset_node
  ${catkin_LIBRARIES}
)

target_link_libraries(exploration_controller_node
  ${catkin_LIBRARIES}
)
