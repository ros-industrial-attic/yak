#include <exploration_controller_node.hpp>

Explorer::Explorer(ros::NodeHandle &nh) {
//  moveit::planning_interface::MoveGroupInterface move_group("manipulator");
//  move_group_ = move_group;
  nbv_client_ = nh.serviceClient<nbv_planner::GetNBV>("/get_nbv");
//  exploration_server_ = nh.advertiseService<std_srvs::Empty>("do_exploration", &Explorer::MoveToNBVs, this);

}

bool Explorer::MoveToNBVs(moveit::planning_interface::MoveGroupInterface &move_group)
{
  /*
   * While "finished" criteria is False:
   * - Get next best view given current information
   * - Move to view
   */
  while (true)
  {
    nbv_planner::GetNBV srv;
    nbv_client_.call(srv);
    if (srv.response.exploration_done) {
      ROS_INFO("Exploration reasonably completed");
      break;
    }
    geometry_msgs::PoseArray move_targets = srv.response.bestViewPose;

    // Go to the best reachable pose
    int currentPoseIndex = 0;
    geometry_msgs::Pose targetPose = move_targets.poses[currentPoseIndex];
    move_group.setPoseTarget(targetPose);
    ROS_INFO_STREAM("Moving to pose: " << targetPose);
    while (!move_group.move())
    {
      currentPoseIndex++;
      if (move_targets.poses.size() == 0) {
        ROS_ERROR("Couldn't reach any of the provided poses!");
        break;
      }
      targetPose = move_targets.poses[currentPoseIndex];
      ROS_INFO_STREAM("Pose not reachable, trying next best pose: " << targetPose);
      move_group.setPoseTarget(targetPose);
    }


    //    geometry_msgs::PoseArray posesToTarget;
    //    geometry_msgs::Pose currentPose = move_group.getCurrentPose().pose;
    //    ROS_INFO_STREAM("Current pose: " << currentPose);
    //    InterpolatePoses(currentPose, targetPose, posesToTarget, 5);

    //    ROS_INFO_STREAM("Moving to pose: " << targetPose);
    //    while (!MoveToPoseSeries(move_group, posesToTarget))
    //    {
    //      currentPoseIndex++;
    //      if (move_targets.poses.size() == 0) {
    //        ROS_ERROR("Couldn't reach any of the provided poses!");
    //        break;
    //      }
    //      targetPose = move_targets.poses[currentPoseIndex];
    //      posesToTarget.poses.clear();
    //      InterpolatePoses(move_group.getCurrentPose().pose, targetPose, posesToTarget, 5);
    //      ROS_INFO_STREAM("Pose not reachable, trying next best pose: " << targetPose);
    //    }
    //  }
  }
}

bool Explorer::MoveToPoseSeries(moveit::planning_interface::MoveGroupInterface &move_group, geometry_msgs::PoseArray &posesIn)
{
  for (int i = 0; i < posesIn.poses.size(); i++){
    move_group.setPoseTarget(posesIn.poses[i]);
    ROS_INFO_STREAM("Intermediate pose: " << posesIn.poses[i]);
    if(!move_group.move())
    {
      ROS_INFO("Couldn't move to that pose!");
      return false;
    }
  }
  return true;
}

bool Explorer::InterpolatePoses(geometry_msgs::Pose start, geometry_msgs::Pose end, geometry_msgs::PoseArray &posesOut, int numSteps)
{
  ROS_INFO("Interpolating poses");
  tf::Transform startTf, endTf;
  tf::poseMsgToTF(start, startTf);
  tf::poseMsgToTF(end, endTf);

  for (float proportion = 0; proportion <= 1.0; proportion+=(1.0/(float)numSteps))
  {
    ROS_INFO_STREAM("Proportion: " << proportion);
    tf::Transform interpolatedTf(startTf.getRotation().slerp(endTf.getRotation(), tfScalar(proportion)), startTf.getOrigin().lerp(endTf.getOrigin(), tfScalar(proportion)));
    geometry_msgs::Pose interpolated;
    tf::poseTFToMsg(interpolatedTf, interpolated);
    posesOut.poses.push_back(interpolated);
    ROS_INFO_STREAM(interpolated);
  }
  return true;
}



int main(int argc, char* argv[])
{
    ros::init(argc, argv, "exploration_controller_node");
    ros::NodeHandle nh;

    moveit::planning_interface::MoveGroupInterface move_group("manipulator_ensenso");
    move_group.setPlanningTime(0.5);

    Explorer explorer(nh);

    ros::AsyncSpinner async_spinner(1);
    async_spinner.start();

    explorer.MoveToNBVs(move_group);

//    ros::waitForShutdown();

    return 0;
}
