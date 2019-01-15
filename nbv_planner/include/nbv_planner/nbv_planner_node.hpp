#include <time.h>

#include <ros/ros.h>
#include "nbv_planner/GetNBV.h"

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

#include <geometry_msgs/Pose.h>

#include <visualization_msgs/Marker.h>


class NBVSolver {
  public:
    NBVSolver(ros::NodeHandle& nh);

    bool GetNBV(nbv_planner::GetNBV::Request &req, nbv_planner::GetNBV::Response &res);

    void GenerateViewPosesSpherical(float distance, int slices, float yawMin, float yawMax, float pitchMin, float pitchMax, tf::Transform &origin, std::list<tf::Transform> &poseList);

    void GenerateViewPosesRandom(int numPoses, float yawMin, float yawMax, float pitchMin, float pitchMax, float dMin, float dMax, std::list<tf::Transform> &poseList);

    int EvaluateCandidateView(tf::Transform pose, octomap::ColorOcTree &tree, octomap::ColorOcTree &unknownTree);

    float RandInRange(float min, float max);

    void Update();

    ros::ServiceServer nbv_server_;

    ros::ServiceClient octomap_client_;

    sensor_msgs::PointCloud2 unknown_leaf_cloud_;
    ros::Publisher unknown_cloud_publisher_;

    visualization_msgs::Marker ray_line_list_;
    visualization_msgs::Marker hit_ray_line_list_;

    ros::Publisher all_ray_pub_;
    ros::Publisher hit_ray_pub_;

    tf::TransformBroadcaster broadcaster_;
    tf::TransformListener listener_;

    int ray_count_;
    int num_pose_slices_;
    float raycast_distance_;

    octomath::Vector3 bound_min_;
    octomath::Vector3 bound_max_;

    std::list<tf::Transform> candidate_poses_;
};
