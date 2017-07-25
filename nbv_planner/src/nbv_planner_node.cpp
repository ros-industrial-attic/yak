#include <nbv_planner_node.hpp>

NBVSolver::NBVSolver(ros::NodeHandle &nh) {
  nbv_server_ = nh.advertiseService("get_nbv", &NBVSolver::GetNBV, this);
  octomap_client_ = nh.serviceClient<octomap_msgs::GetOctomap>("/octomap_full");
  unknown_cloud_publisher_ = nh.advertise<sensor_msgs::PointCloud2>("my_unknown_cloud",1);
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

  octomap::ColorOcTree* my_map = (octomap::ColorOcTree*)abstract_tree;
  octomap::ColorOcTree tree = *my_map;

  int numLeaves = tree.calcNumNodes();
  ROS_INFO_STREAM("Number of nodes: " << numLeaves);


  std::list<octomath::Vector3> unknownLeafs;


  octomath::Vector3 boundMin(-0.3, -0.5, 0.1);
  octomath::Vector3 boundMax(0.2, 0.0, 0.5);
  tree.getUnknownLeafCenters(unknownLeafs, boundMin, boundMax, 16);
  ROS_INFO_STREAM("Number of unknown leaves: " << unknownLeafs.size());

  octomath::Vector3 first = unknownLeafs.front();
  ROS_INFO_STREAM("First coords: " << first.x() << " " << first.y() << " " << first.z());

  pcl::PointCloud<pcl::PointXYZ> pclCloud;

  for (std::list<octomath::Vector3>::const_iterator it = unknownLeafs.begin(); it != unknownLeafs.end(); ++it) {
      ROS_INFO_STREAM("Unknown point at " << it->x() << " " << it->x() << " " << it->z());
      pcl::PointXYZ newPoint(it->x(), it->y(), it->z());
      pclCloud.push_back(newPoint);
  }


  sensor_msgs::PointCloud2 unknownLeafCloud;
  pcl::toROSMsg(pclCloud, unknownLeafCloud);

  ROS_INFO("converted leaf list to point cloud");

//  ROS_INFO_STREAM("Cloud width: " << unknownLeafCloud.width);

  unknownLeafCloud.header.stamp = ros::Time::now();
  unknownLeafCloud.header.frame_id = "map";
  unknown_cloud_publisher_.publish(unknownLeafCloud);
  ROS_INFO("Published updated point cloud");




  res.value = true;

  abstract_tree->clear();

  return true;
}

void NBVSolver::GenerateViewPoses(float distance, int slices, std::list<tf::Transform> &poseList) {
  tf::Transform offset(tf::Quaternion(0, 0, 0, 1), tf::Vector3(-distance, 0, 0));

//  std::list<tf::Transform> poses;

  for (float angle = 0; angle < 2*M_PI; angle += 2*M_PI/slices) {
    ROS_INFO_STREAM("Rotation by " << angle);
    for (float elevation = 0; elevation < M_PI/2; elevation += (M_PI/2)/(slices/2)) {
//      tf::Quaternion quat(tf::Vector3(0,0,1), tfScalar(angle));
      tf::Transform yaw(tf::Quaternion(tf::Vector3(0,0,1), tfScalar(angle)), tf::Vector3(0,0,0));
      tf::Transform pitch(tf::Quaternion(tf::Vector3(0,1,0), tfScalar(elevation)), tf::Vector3(0,0,0));
      poseList.push_back(yaw*pitch*offset);
    }

  }
}

int NBVSolver::EvaluateCandidateView(tf::Transform &pose, octomap::ColorOcTree &tree) {
  float fov = 45.0;
  int rayCount = 16;
  std::list<tf::Transform> rayPoses;
  for (float angleWidth = -fov/2; angleWidth < fov/2; angleWidth += fov/rayCount) {
    for (float angleHeight = -fov/2; angleHeight < fov/2; angleHeight += fov/rayCount) {
      tf::Quaternion rotation(tfScalar(angleWidth), tfScalar(angleHeight), tfScalar(0));
      rayPoses.push_back(tf::Transform(rotation, tf::Vector3(0,0,0))*pose);
    }
  }

  for (std::list<tf::Transform>::const_iterator it = rayPoses.begin(); it != rayPoses.end(); ++it) {
//    tree.castRay(octomap::po)
    // TODO: Generate poses in here. Raycast from poses into tree without ignoring unknown nodes.
    // Use resulting coordinates to search tree for hit nodes and return which ones or how many were unknown. This should avoid occlusion/shadow problem.
    // A reasonable metric for view quality could be to maximize the unknown node count in a given view.
  }

  return 0;
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "nbv_planner_node");
    ros::NodeHandle nh;

    NBVSolver solver(nh);

    ROS_INFO("Making candidate poses...");
    std::list<tf::Transform> poses;
    solver.GenerateViewPoses(0.75, 4, poses);

    ROS_INFO("Made some poses");

    ros::Rate rate(10.0);

    while (ros::ok()){
      int count = 0;
      for (std::list<tf::Transform>::const_iterator it = poses.begin(); it != poses.end(); ++it) {
        tf::StampedTransform currentStampedPose(*it, ros::Time::now(), "volume_pose", "candidate" + std::to_string(count));
        solver.broadcaster_.sendTransform(currentStampedPose);
        count++;
      }
      rate.sleep();
      ros::spinOnce();
    }

    return 0;
}
