#include <nbv_planner_node.hpp>

NBVSolver::NBVSolver(ros::NodeHandle &nh)
{
  nbv_server_ = nh.advertiseService("get_nbv", &NBVSolver::GetNBV, this);
  octomap_client_ = nh.serviceClient<octomap_msgs::GetOctomap>("/octomap_full");
  unknown_cloud_publisher_ = nh.advertise<sensor_msgs::PointCloud2>("my_unknown_cloud",1);

  float bound_min_x, bound_min_y, bound_min_z, bound_max_x, bound_max_y, bound_max_z, voxel_res;
  int dims_x, dims_y, dims_z;

//  double volume_dim_x, volume_dim_y, volume_dim_z;

  voxel_res = 0.001;
  dims_x = 480;
  dims_y = 480;
  dims_z = 256;

  // Can't load parameters from other nodes like this!
//  nh.getParam("volume_dims_x", dims_x);
//  nh.getParam("volume_dims_y", dims_y);
//  nh.getParam("volume_dims_z", dims_z);
//  nh.getParam("volume_resolution", voxel_res);
  if (!nh.hasParam("/nbv_planner_node/volume_dim_x"))
  {
      ROS_ERROR("Param not found!");
  }

  // TODO: Fix param loading
//  nh.param<double>("/nbv_planner_node/volume_dim_x", volume_dim_x);
//  nh.param<double>("/nbv_planner_node/volume_dim_y", volume_dim_y);
//  nh.param<double>("/nbv_planner_node/volume_dim_z", volume_dim_z);

//  ROS_INFO_STREAM(volume_dim_x << " " << volume_dim_y << " " << volume_dim_z);

  bound_min_z = bound_min_x = bound_min_y = 0.0;

//  bound_min_x = -(float)volume_dim_x;
//  bound_min_y = -(float)volume_dim_y;
//  bound_max_z = (float)volume_dim_z;


  bound_max_x = voxel_res*(float)dims_x;
  bound_max_y = voxel_res*(float)dims_y;
  bound_max_z = voxel_res*(float)dims_z;

  bound_min_ = octomath::Vector3(bound_min_x, bound_min_y, bound_min_z);
  bound_max_ = octomath::Vector3(bound_max_x, bound_max_y, bound_max_z);

  ROS_INFO_STREAM("Evaluating volume from " << bound_min_ << " to " << bound_max_);

//  nh.param<int>("num_pose_slices", num_pose_slices_);
//  nh.param<int>("ray_count", ray_count_);
//  nh.param<float>("raycast_distance", raycast_distance_);

  num_pose_slices_ = 8;
  ray_count_ = 15;
  raycast_distance_ = 0.5;

  ROS_INFO_STREAM(num_pose_slices_ << " " << ray_count_ << " " << raycast_distance_);

//  bound_min_ = octomath::Vector3(-0.5, -0.5, 0.0);
//  bound_max_ = octomath::Vector3(0.0, 0.0, 0.5);

  all_ray_pub_ = nh.advertise<visualization_msgs::Marker>("all_rays", 10);

  ray_line_list_.header.frame_id = hit_ray_line_list_.header.frame_id = "/volume_pose";
  ray_line_list_.scale.x = hit_ray_line_list_.scale.x = 0.0001;
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
//  GenerateViewPosesSpherical(raycast_distance_, num_pose_slices_, orbitCenter, poses);
//  GenerateViewPosesSpherical(raycast_distance_, num_pose_slices_, -M_PI/8, M_PI/8, 3*M_PI/8, M_PI/2, orbitCenter, poses);

  GenerateViewPosesRandom(64, 0, 2*M_PI, M_PI/16, M_PI/2, 0.5, 0.75, poses);
//  GenerateViewPosesRandom(1, M_PI, M_PI, M_PI/2, M_PI/2, 0.6, 0.6, poses);

  ray_line_list_.points.clear();
  hit_ray_line_list_.points.clear();

  std::vector<int> viewMetrics;
  std::vector<std::tuple<tf::Transform, int>> viewsWithMetrics;

  for (std::list<tf::Transform>::const_iterator it = poses.begin(); it != poses.end(); ++it)
  {
    candidate_poses_.push_back(*it);
    int fitness = EvaluateCandidateView(*it, tree, unknownTree);
    ROS_INFO_STREAM("Casts from this view hit " << fitness << " unseen voxels.");
    viewMetrics.push_back(fitness);
    viewsWithMetrics.push_back(std::tuple<tf::Transform, int>(*it, fitness));
  }

  std::vector<int>::iterator result;

  result = std::max_element(viewMetrics.begin(), viewMetrics.end());
  int indexOfMax = std::distance(viewMetrics.begin(), result);

  std::list<tf::Transform>::iterator poseIt = candidate_poses_.begin();
  std::advance(poseIt, indexOfMax);

  ROS_INFO_STREAM("Best view is # " << indexOfMax);


//  std::sort(viewsWithMetrics.begin(), viewsWithMetrics.end(), NBVSolver::ComparePosesWithMetrics());
  std::sort( viewsWithMetrics.begin(), viewsWithMetrics.end(), [ ]( const std::tuple<tf::Transform, int>& a, const std::tuple<tf::Transform, int>& b )
  {
     return std::get<1>(a) > std::get<1>(b);
  });

  tf::StampedTransform base_to_volume;
  listener_.waitForTransform("base_link", "volume_pose", ros::Time::now(), ros::Duration(0.5));
  listener_.lookupTransform("base_link", "volume_pose", ros::Time(0), base_to_volume);

  for (int i = 0; i < viewsWithMetrics.size(); i++) {
    ROS_INFO_STREAM(std::get<1>(viewsWithMetrics[i]));
    geometry_msgs::Pose aPose;
    tf::poseTFToMsg(tf::Transform(base_to_volume.getRotation(), base_to_volume.getOrigin()) * std::get<0>(viewsWithMetrics[i]), aPose);
    res.bestViewPose.poses.push_back(aPose);
  }

//  best_pose_ = *poseIt;


  // Need to return pose relative to base_link for robot planning. Right now it's relative to volume_pose.

//  tf::poseTFToMsg(tf::Transform(base_to_volume.getRotation(), base_to_volume.getOrigin()) * (*poseIt), res.bestViewPose);

  ROS_INFO_STREAM("Best pose (base-relative): " << res.bestViewPose.poses.front());

  if (*result <= 5)
  {
    // If very few unknown voxels are visible from the best view, then it's probably not worth exploring any more.
    res.exploration_done = true;
  }
  else
  {
    res.exploration_done = false;
  }



  abstract_tree->clear();

  return true;
}

bool NBVSolver::ComparePosesWithMetrics(const std::tuple<tf::Transform, int>& a, const std::tuple<tf::Transform, int>& b) {
  int first = std::get<1>(a);
  int second = std::get<1>(b);
  return first > second;
}

void NBVSolver::GenerateViewPosesSpherical(float distance, int slices, float yawMin, float yawMax, float pitchMin, float pitchMax, tf::Transform &origin, std::list<tf::Transform> &poseList)
{
  tf::Transform offset(tf::Quaternion(tf::Vector3(0,0,1), tfScalar(0)), tf::Vector3(-distance, 0, 0));

  for (float angle = yawMin; angle <= yawMax; angle += (yawMax - yawMin)/(slices-1))
  {
    for (float elevation = pitchMin; elevation < pitchMax; elevation += (pitchMax - pitchMin)/slices)
    {
      tf::Transform yaw(tf::Quaternion(tf::Vector3(0,0,1), tfScalar(angle)), tf::Vector3(0,0,0));
      tf::Transform pitch(tf::Quaternion(tf::Vector3(0,1,0), tfScalar(elevation)), tf::Vector3(0,0,0));
      poseList.push_back(origin*yaw*pitch*offset);

//      tf::Transform pitch(tf::Quaternion(tf::Vector3(0,1,0), tfScalar(-elevation + 3*M_PI/2)), tf::Vector3(0,0,0));
//      tf::Transform yaw(tf::Quaternion(tf::Vector3(0,0,1), tfScalar(angle)), tf::Vector3(0,0,0));
//      tf::Transform roll(tf::Quaternion(tf::Vector3(0,0,1), tfScalar(-M_PI/2)), tf::Vector3(0,0,0));

//      poseList.push_back(origin*yaw*pitch*roll*offset);
    }
  }
}

void NBVSolver::GenerateViewPosesRandom(int numPoses, float yawMin, float yawMax, float pitchMin, float pitchMax, float dMin, float dMax, std::list<tf::Transform> &poseList)
{
  while (poseList.size() < numPoses)
  {
//    float originX = bound_min_.x() + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(bound_max_.x()- bound_min_.x())));
//    float originY = bound_min_.y() + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(bound_max_.y()- bound_min_.y())));
//    float originZ = bound_min_.z() + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(bound_max_.z()- bound_min_.z())));

    float originX = this->RandInRange(bound_min_.x(), bound_max_.x());
    float originY = this->RandInRange(bound_min_.y(), bound_max_.y());
//    float originZ = this->RandInRange(bound_min_.z(), bound_max_.z());
    float originZ = (bound_max_.z() + bound_min_.z())/2.0;

    tf::Transform origin(tf::Quaternion(tf::Vector3(0,0,1), tfScalar(0)), tf::Vector3(originX, originY, originZ));

//    float distance = dMin + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(dMax - dMin)));
    float distance = this->RandInRange(dMin, dMax);
    tf::Transform offset(tf::Quaternion(tf::Vector3(0,0,1), tfScalar(0)), tf::Vector3(0, 0, -distance));

//    float angle = yawMin + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(yawMax - yawMin)));
//    float elevation = pitchMin + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(pitchMax - pitchMin)));
    float angle = this->RandInRange(yawMin, yawMax);
    float elevation = this->RandInRange(pitchMin, pitchMax);

    tf::Transform pitch(tf::Quaternion(tf::Vector3(0,1,0), tfScalar(-elevation + 3*M_PI/2)), tf::Vector3(0,0,0));
    tf::Transform yaw(tf::Quaternion(tf::Vector3(0,0,1), tfScalar(angle)), tf::Vector3(0,0,0));
    tf::Transform roll(tf::Quaternion(tf::Vector3(0,0,1), tfScalar(-M_PI/2)), tf::Vector3(0,0,0));

//    poseList.push_back(origin*yaw*pitch*offset);
    poseList.push_back(origin*yaw*pitch*roll*offset);
  }
  //TODO: Not yet implemented
}

float NBVSolver::RandInRange(float min, float max) {
  return min + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(max - min)));
}

int NBVSolver::EvaluateCandidateView(tf::Transform pose, octomap::ColorOcTree &tree, octomap::ColorOcTree &unknownTree)
{
  float fov = M_PI/6.0;

  int unknownCount = 0;

  std::list<octomath::Vector3> rays;
//  std::list<octomath::Vector3> hitUnknowns;
  for (float angleWidth = -fov/2.0; angleWidth <= fov/2.0; angleWidth += fov/ray_count_)
  {
    for (float angleHeight = -fov/2.0; angleHeight <= fov/2.0; angleHeight += fov/ray_count_)
    {
      tf::Matrix3x3 rotationMat;
      rotationMat.setRPY(tfScalar(angleHeight), tfScalar(angleWidth), tfScalar(0.0));

      tf::Quaternion rotation;
      rotationMat.getRotation(rotation);

      tf::Transform rayTf(tf::Quaternion(tf::Vector3(0,0,1), tfScalar(0)), tf::Vector3(0,0,1));
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
        hit_ray_line_list_.points.push_back(octomap::pointOctomapToMsg(hitUnknown));
      }
    }
    else if (!raycastToKnown && raycastToUnknown)
    {
      // Unknown voxel occludes only free space
      unknownCount++;
      hit_ray_line_list_.points.push_back(p);
      hit_ray_line_list_.points.push_back(octomap::pointOctomapToMsg(hitUnknown));
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

    std::srand(std::time(0));

    NBVSolver solver(nh);

    ros::Rate rate(10.0);

    while (ros::ok()){
      solver.Update();
      rate.sleep();
      ros::spinOnce();
    }

    return 0;
}
