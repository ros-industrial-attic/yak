#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <nbv_planner/GetNBV.h>
#include <nbv_planner/GetNBVRequest.h>
#include <nbv_planner/GetNBVResponse.h>

#include <tf/tf.h>

#include <std_srvs/Empty.h>
#include <std_srvs/EmptyRequest.h>
#include <std_srvs/EmptyResponse.h>

class Explorer {
public:
  Explorer(ros::NodeHandle &nh);

  bool MoveToNBVs(moveit::planning_interface::MoveGroupInterface &move_group);

  bool InterpolatePoses(geometry_msgs::Pose start, geometry_msgs::Pose end, geometry_msgs::PoseArray &poses, int numSteps);

  bool MoveToPoseSeries(moveit::planning_interface::MoveGroupInterface &move_group, geometry_msgs::PoseArray &posesIn);

//  moveit::planning_interface::MoveGroupInterface move_group_;
  ros::ServiceClient nbv_client_;
//  ros::ServiceServer exploration_server_;

private:
};
