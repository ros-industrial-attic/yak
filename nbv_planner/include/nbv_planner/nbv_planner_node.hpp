#include <ros/ros.h>
#include <nbv_planner/GetNBV.h>
#include <nbv_planner/GetNBVRequest.h>
#include <nbv_planner/GetNBVResponse.h>

#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/GetOctomapRequest.h>
#include <octomap_msgs/GetOctomapResponse.h>

#include <octomap_msgs/conversions.h>

#include <octomap/octomap.h>

#include <octomap_ros/conversions.h>

#include <octomap/math/Vector3.h>
//#include <octomap/ColorOcTree.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include <math.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/pcl_base.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <moveit/move_group_interface/move_group_interface.h>


class NBVSolver {
  public:
    NBVSolver(ros::NodeHandle& nh);

    bool GetNBV(nbv_planner::GetNBVRequest& req, nbv_planner::GetNBVResponse& res);

    void GenerateViewPoses(float distance, int slices, std::list<tf::Transform> &poseList);

    int EvaluateCandidateViews(std::list<tf::Transform> &poseList);

    ros::ServiceServer nbv_server_;

    ros::ServiceClient octomap_client_;

    sensor_msgs::PointCloud2 unknown_leaf_cloud_;
    ros::Publisher unknown_cloud_publisher_;

    tf::TransformBroadcaster broadcaster_;

//    octomap::OcTree octree_;

 // private:

};
