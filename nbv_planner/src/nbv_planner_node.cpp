#include <nbv_planner_node.hpp>

NBVSolver::NBVSolver(ros::NodeHandle &nh)
{
  nbv_server_ = nh.advertiseService("get_nbv", &NBVSolver::GetNBV, this);
  octomap_client_ = nh.serviceClient<octomap_msgs::GetOctomap>("/octomap_full");
  unknown_cloud_publisher_ = nh.advertise<sensor_msgs::PointCloud2>("my_unknown_cloud",1);
  bound_min_ = octomath::Vector3(-0.5, -0.5, 0.0);
  bound_max_ = octomath::Vector3(0.0, 0.0, 0.5);
  all_ray_pub_ = nh.advertise<visualization_msgs::Marker>("all_rays", 10);

  ray_line_list_.header.frame_id = hit_ray_line_list_.header.frame_id = "/volume_pose";
  ray_line_list_.scale.x = hit_ray_line_list_.scale.x = 0.001;
  ray_line_list_.action = hit_ray_line_list_.action = visualization_msgs::Marker::ADD;
  ray_line_list_.pose.orientation.w = hit_ray_line_list_.pose.orientation.w = 1.0;
  ray_line_list_.type = hit_ray_line_list_.type = visualization_msgs::Marker::LINE_LIST;
  ray_line_list_.color.b = 1.0;
  hit_ray_line_list_.color.r = 1.0;
  ray_line_list_.color.a = hit_ray_line_list_.color.a = 1.0;

  hit_ray_pub_ = nh.advertise<visualization_msgs::Marker>("hit_rays", 10);
}

bool NBVSolver::GetNBV(nbv_planner::GetNBVRequest& req, nbv_planner::GetNBVResponse& res)
{
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

  octomap::ColorOcTree unknownTree(tree.getResolution());

  int numLeaves = tree.calcNumNodes();
  ROS_INFO_STREAM("Number of nodes: " << numLeaves);

  std::list<octomath::Vector3> unknownLeafs;


//  octomath::Vector3 boundMin(-0.5, -0.5, 0.0);
//  octomath::Vector3 boundMax(0.0, 0.0, 0.5);
  tree.getUnknownLeafCenters(unknownLeafs, bound_min_, bound_max_, 0);
  ROS_INFO_STREAM("Number of unknown leaves: " << unknownLeafs.size());

  octomath::Vector3 first = unknownLeafs.front();
  ROS_INFO_STREAM("First coords: " << first.x() << " " << first.y() << " " << first.z());

  pcl::PointCloud<pcl::PointXYZ> pclCloud;

  for (std::list<octomath::Vector3>::const_iterator it = unknownLeafs.begin(); it != unknownLeafs.end(); ++it)
  {
//      ROS_INFO_STREAM("Unknown point at " << it->x() << " " << it->x() << " " << it->z());
      pcl::PointXYZ newPoint(it->x(), it->y(), it->z());
      pclCloud.push_back(newPoint);
      unknownTree.updateNode(it->x(), it->y(), it->z(), true);
  }


  sensor_msgs::PointCloud2 unknownLeafCloud;
  pcl::toROSMsg(pclCloud, unknownLeafCloud);

  ROS_INFO("converted leaf list to point cloud");

//  ROS_INFO_STREAM("Cloud width: " << unknownLeafCloud.width);

  unknownLeafCloud.header.stamp = ros::Time::now();
  unknownLeafCloud.header.frame_id = "map";
  unknown_cloud_publisher_.publish(unknownLeafCloud);
  ROS_INFO("Published updated point cloud");

  ROS_INFO("Making candidate poses...");
  candidate_poses_.clear();
  std::list<tf::Transform> poses;
  tf::Transform orbitCenter(tf::Quaternion(0,0,0,1), tf::Vector3((bound_min_.x()-bound_max_.x())/2, (bound_min_.y()-bound_max_.y())/2, (bound_max_.z()-bound_min_.z())/2));
  GenerateViewPoses(0.75, 4, orbitCenter, poses);

  ray_line_list_.points.clear();
  hit_ray_line_list_.points.clear();

  std::vector<int> viewMetrics;

  for (std::list<tf::Transform>::const_iterator it = poses.begin(); it != poses.end(); ++it)
  {
    candidate_poses_.push_back(*it);
    int fitness = EvaluateCandidateView(*it, tree, unknownTree);
    ROS_INFO_STREAM("Casts from this view hit " << fitness << " unseen voxels.");
    viewMetrics.push_back(fitness);
  }

  std::vector<int>::iterator result;

  result = std::max_element(viewMetrics.begin(), viewMetrics.end());
  int indexOfMax = std::distance(viewMetrics.begin(), result);

  std::list<tf::Transform>::iterator poseIt = candidate_poses_.begin();
  std::advance(poseIt, indexOfMax);

  ROS_INFO_STREAM("Best view is # " << indexOfMax);


  tf::poseTFToMsg(*poseIt, res.bestViewPose);


  res.value = true;

  abstract_tree->clear();

  return true;
}

void NBVSolver::GenerateViewPoses(float distance, int slices, tf::Transform &origin, std::list<tf::Transform> &poseList)
{
  tf::Transform offset(tf::Quaternion(tf::Vector3(0,0,1), tfScalar(0)), tf::Vector3(-distance, 0, 0));

  for (float angle = 0; angle < 2*M_PI; angle += 2*M_PI/slices)
  {
    for (float elevation = 0; elevation < M_PI/2; elevation += (M_PI/2)/(slices/2))
    {
      tf::Transform yaw(tf::Quaternion(tf::Vector3(0,0,1), tfScalar(angle)), tf::Vector3(0,0,0));
      tf::Transform pitch(tf::Quaternion(tf::Vector3(0,1,0), tfScalar(elevation)), tf::Vector3(0,0,0));
      poseList.push_back(origin*yaw*pitch*offset);
    }
  }
}

int NBVSolver::EvaluateCandidateView(tf::Transform pose, octomap::ColorOcTree &tree, octomap::ColorOcTree &unknownTree)
{
  float fov = M_PI/4.0;
  int rayCount = 7;

  int unknownCount = 0;

  std::list<octomath::Vector3> rays;
//  std::list<octomath::Vector3> hitUnknowns;
  for (float angleWidth = -fov/2.0; angleWidth <= fov/2.0; angleWidth += fov/rayCount)
  {
    for (float angleHeight = -fov/2.0; angleHeight <= fov/2.0; angleHeight += fov/rayCount)
    {
      tf::Matrix3x3 rotationMat;
      rotationMat.setRPY(tfScalar(0.0), tfScalar(angleHeight), tfScalar(angleWidth));

      tf::Quaternion rotation;
      rotationMat.getRotation(rotation);

      tf::Transform rayTf(tf::Quaternion(tf::Vector3(0,0,1), tfScalar(0)), tf::Vector3(1,0,0));
      tf::Transform rayRotated = tf::Transform(pose.getRotation(), tf::Vector3(0,0,0)) * tf::Transform(rotation, tf::Vector3(0,0,0)) * rayTf;
      tf::Vector3 ray = rayRotated.getOrigin();
      rays.push_back(octomap::pointTfToOctomap(ray));
    }
  }

  for (std::list<octomath::Vector3>::const_iterator it = rays.begin(); it != rays.end(); ++it)
  {
    octomath::Vector3 origin = octomap::pointTfToOctomap(pose.getOrigin());
    octomath::Vector3 hitKnown;
    octomath::Vector3 hitUnknown;

    geometry_msgs::Point p;
    p.x = origin.x();
    p.y = origin.y();
    p.z = origin.z();
    ray_line_list_.points.push_back(p);

    geometry_msgs::Point q;
    q.x = origin.x() + it->x();
    q.y = origin.y() + it->y();
    q.z = origin.z() + it->z();
    ray_line_list_.points.push_back(q);

//    ROS_INFO_STREAM("Origin: " << origin << " direction: " << *it);
    bool raycastToKnown = tree.castRay(origin, *it, hitKnown, true, 1.0);
    bool raycastToUnknown = unknownTree.castRay(origin, *it, hitUnknown, true, 1.0);
    if (raycastToKnown && raycastToUnknown)
    {
      double distanceKnown = octomath::Vector3(hitKnown - origin).norm();
      double distanceUnknown = octomath::Vector3(hitUnknown - origin).norm();
      if (distanceUnknown < distanceKnown)
      {
        // Unknown voxel is nearer than an occupied voxel behind it
        unknownCount++;
        hit_ray_line_list_.points.push_back(p);
        hit_ray_line_list_.points.push_back(q);
      }
    }
    else if (!raycastToKnown && raycastToUnknown)
    {
      // Unknown voxel occludes only free space
      unknownCount++;
      hit_ray_line_list_.points.push_back(p);
      hit_ray_line_list_.points.push_back(q);
    }
    // Otherwise, there's either only free space up to a known occupied surface, or there's free space up to the edge of the volume bounds.
    // In either case, there's nothing new for us to learn along that ray.
  }
  return unknownCount;
}

void NBVSolver::Update()
{
  int count = 0;
  for (std::list<tf::Transform>::iterator it = candidate_poses_.begin(); it != candidate_poses_.end(); ++it, count++)
  {
    tf::StampedTransform transformStamped(*it, ros::Time::now(), "volume_pose", "candidate" + std::to_string(count));
    broadcaster_.sendTransform(transformStamped);

  }
  ray_line_list_.header.stamp = ros::Time::now();
  all_ray_pub_.publish(ray_line_list_);
  hit_ray_line_list_.header.stamp = ros::Time::now();
  hit_ray_pub_.publish(hit_ray_line_list_);
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "nbv_planner_node");
    ros::NodeHandle nh;

    NBVSolver solver(nh);

    ros::Rate rate(10.0);

    while (ros::ok()){
      solver.Update();
      rate.sleep();
      ros::spinOnce();
    }

    return 0;
}
