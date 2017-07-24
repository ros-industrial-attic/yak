#include <nbv_planner_node.hpp>

NBVSolver::NBVSolver(ros::NodeHandle &nh) {
  nbv_server_ = nh.advertiseService("get_nbv", &NBVSolver::GetNBV, this);
  octomap_client_ = nh.serviceClient<octomap_msgs::GetOctomap>("/octomap_full");
}

bool NBVSolver::GetNBV(nbv_planner::GetNBVRequest& req, nbv_planner::GetNBVResponse& res) {
  ROS_INFO("Attempting to get octomap");
  octomap_msgs::GetOctomap srv;
  if (!octomap_client_.call(srv))
  {
    ROS_INFO("Couldn't get octomap");
    return false;
  }

  ROS_INFO("Got octomap!");



octomap::AbstractOcTree* abstract_tree = octomap_msgs::fullMsgToMap(srv.response.map);

octomap::OcTree* my_map = (octomap::OcTree*)abstract_tree;
octomap::OcTree tree = *my_map;

//tree.

//  for (octomap::OcTree::iterator it = tree->begin(16), end = tree->end(); it != end; ++it)
//  {
//      ROS_INFO("Hit a leaf");
//  }

  return true;
}

void GenerateViews() {

}

int EvaluateCandidateView() {

  return 0;
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "nbv_planner_node");
    ros::NodeHandle nh;

    NBVSolver solver(nh);

    while (ros::ok()){
        ros::spinOnce();
    }

    return 0;
}
