#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <nbv_planner/GetNBV.h>
#include <nbv_planner/GetNBVRequest.h>
#include <nbv_planner/GetNBVResponse.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <std_srvs/Empty.h>
#include <std_srvs/EmptyRequest.h>
#include <std_srvs/EmptyResponse.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit_msgs/Constraints.h>
#include <moveit/kinematic_constraints/kinematic_constraint.h>
#include <moveit/kinematic_constraints/utils.h>

#include <shape_msgs/SolidPrimitive.h>

class Explorer {
public:
  Explorer(ros::NodeHandle &nh);

  bool MoveToNBVs(moveit::planning_interface::MoveGroupInterface &move_group);

//  moveit::planning_interface::MoveGroupInterface move_group_;
  ros::ServiceClient nbv_client_;
  tf::TransformListener tfListener_;
  ros::NodeHandle node_handle_;

//  ros::ServiceServer exploration_server_;

private:
};
