#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <nbv_planner/GetNBV.h>
#include <nbv_planner/GetNBVRequest.h>
#include <nbv_planner/GetNBVResponse.h>

#include <std_srvs/Empty.h>
#include <std_srvs/EmptyRequest.h>
#include <std_srvs/EmptyResponse.h>

class Explorer {
public:
  Explorer(ros::NodeHandle &nh);

  bool MoveToNBVs(moveit::planning_interface::MoveGroupInterface &move_group);

//  moveit::planning_interface::MoveGroupInterface move_group_;
  ros::ServiceClient nbv_client_;
//  ros::ServiceServer exploration_server_;

private:
};
