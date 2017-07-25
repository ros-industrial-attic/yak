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
  tree.getUnknownLeafCenters(unknownLeafs, boundMin, boundMax, 15);
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

//  sensor_msgs::PointField fieldX, fieldY, fieldZ;
//  fieldX.name = "x";
//  unknownLeafCloud.fields.push_back(fieldX);
//  fieldY.name = "y";
//  unknownLeafCloud.fields.push_back(fieldY);
//  fieldZ.name = "z";
//  unknownLeafCloud.fields.push_back(fieldZ);
//  unknownLeafCloud.fields[1].name = "y";
//  unknownLeafCloud.fields[2].name = "z";

  // Fails here
  // pointcloud2 needs to be initialized with three fields: x, y, z
  // Thit might be a bad way to do this, since it segfaults if the field names are found but nothing else is set.
//  octomap::pointsOctomapToPointCloud2(unknownLeafs, unknownLeafCloud);


//  sensor_msgs::PointCloud2Modifier pcd_modifier(unknownLeafCloud);
//  pcd_modifier.resize(unknownLeafs.size());

  // // Can't find field "x" in the point cloud
//  sensor_msgs::PointCloud2Iterator<float> iter_x(unknownLeafCloud, "x");
//  sensor_msgs::PointCloud2Iterator<float> iter_y(unknownLeafCloud, "y");
//  sensor_msgs::PointCloud2Iterator<float> iter_z(unknownLeafCloud, "z");

//  for (std::list<octomath::Vector3>::const_iterator it = unknownLeafs.begin(); it != unknownLeafs.end(); ++it, ++iter_x, ++iter_y, ++iter_z) {
//    ROS_INFO_STREAM("Unknown point at " << it->x() << " " << it->x() << " " << it->z());
//    *iter_x = it->x();
//    *iter_y = it->y();
//    *iter_z = it->z();
//}

  ROS_INFO("converted leaf list to point cloud");

//  ROS_INFO_STREAM("Cloud width: " << unknownLeafCloud.width);

  unknownLeafCloud.header.stamp = ros::Time::now();
  unknownLeafCloud.header.frame_id = "map";
  unknown_cloud_publisher_.publish(unknownLeafCloud);
  ROS_INFO("Published updated point cloud");


//  for (octomap::ColorOcTree::iterator it = tree.begin(), end = tree.end(); it != end; ++it)
//  {
//    // Want to get value stored in node
//      ROS_INFO_STREAM("Hit a leaf: " << it.getCoordinate());



//  }

//  octomath::Vector3 point;
//  octomath::Vector3 direction(-0.25,0.0,1.0);
//  if (tree.castRay(octomath::Vector3(0,0,0), direction, point)){
//    ROS_INFO_STREAM("Raycast hit something at " << point.x() << " , " << point.y() << " , " << point.z());
//  }



  res.value = true;

  abstract_tree->clear();

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
