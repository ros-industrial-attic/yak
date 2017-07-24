#include <ros/ros.h>
#include <nbv_planner/GetNBV.h>
#include <nbv_planner/GetNBVRequest.h>
#include <nbv_planner/GetNBVResponse.h>

#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/GetOctomapRequest.h>
#include <octomap_msgs/GetOctomapResponse.h>

#include <octomap_msgs/conversions.h>

#include <octomap/octomap.h>
//#include <octomap/ColorOcTree.h>

class NBVSolver {
  public:
    NBVSolver(ros::NodeHandle& nh);

    bool GetNBV(nbv_planner::GetNBVRequest& req, nbv_planner::GetNBVResponse& res);

    void GenerateViews();

    int EvaluateCandidateView() {

    }

    ros::ServiceServer nbv_server_;

    ros::ServiceClient octomap_client_;

//    octomap::OcTree octree_;

 // private:

};
