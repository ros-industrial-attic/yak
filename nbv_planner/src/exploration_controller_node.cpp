#include <exploration_controller_node.hpp>

Explorer::Explorer(ros::NodeHandle &nh) {
//  moveit::planning_interface::MoveGroupInterface move_group("manipulator");
//  move_group_ = move_group;
  nbv_client_ = nh.serviceClient<nbv_planner::GetNBV>("/get_nbv");
//  exploration_server_ = nh.advertiseService<std_srvs::Empty>("do_exploration", &Explorer::MoveToNBVs, this);

}

bool Explorer::MoveToNBVs(moveit::planning_interface::MoveGroupInterface &move_group) {
  /*
   * While "finished" criteria is False:
   * - Get next best view given current information
   * - Move to view
   */

  nbv_planner::GetNBV srv;

  while (true) {
    nbv_client_.call(srv);
    if (srv.response.exploration_done) {
//      res.success = true;
      ROS_INFO("Exploration done!");
      break;
    }
    geometry_msgs::Pose move_target = srv.response.bestViewPose;
    ROS_INFO_STREAM("Exploring towards pose " << srv.response.bestViewPose);
    move_group.setPoseTarget(move_target);
    ROS_INFO("Moving...");
    move_group.move();
    ROS_INFO("Done moving");
  }
  return true;
}



int main(int argc, char* argv[])
{
    ros::init(argc, argv, "exploration_controller_node");
    ros::NodeHandle nh;

    moveit::planning_interface::MoveGroupInterface move_group("manipulator_ensenso");

    Explorer explorer(nh);


    explorer.MoveToNBVs(move_group);

    ros::AsyncSpinner async_spinner(1);
    async_spinner.start();

    ros::waitForShutdown();

    return 0;
}
