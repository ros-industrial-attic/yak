#include <exploration_controller_node.hpp>

Explorer::Explorer(ros::NodeHandle &nh) {
//  moveit::planning_interface::MoveGroupInterface move_group("manipulator");
//  move_group_ = move_group;
  nbv_client_ = nh.serviceClient<nbv_planner::GetNBV>("/get_nbv");
  node_handle_ = nh;
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
    // Make a new call to the GetNBV service to get a list of potentially-good poses to move towards.
    nbv_planner::GetNBV srv;
    nbv_client_.call(srv);
    if (srv.response.exploration_done) {
      ROS_INFO("Exploration reasonably completed");
      break;
    }
    geometry_msgs::PoseArray move_targets = srv.response.bestViewPose;

    // Start with the most highly ranked pose
    int currentPoseIndex = 0;
    geometry_msgs::Pose targetPose = move_targets.poses[currentPoseIndex];


    // Get the position of the TSDF volume to set constraint on camera orientation.
    tf::StampedTransform baseToVolume;
    tfListener_.waitForTransform("base_link", "volume_pose", ros::Time::now(), ros::Duration(0.5));
    tfListener_.lookupTransform("base_link", "volume_pose", ros::Time(0), baseToVolume);

    moveit_msgs::Constraints cameraPoseConstraints;

    // First constraint: keep a TF frame ~60cm in front of the camera within a box around the TSDF volume
    // This will keep the camera pointed more or less at the volume to be imaged.
    moveit_msgs::PositionConstraint volumeBoxPosConstraint;

    geometry_msgs::Quaternion volumePoseQuat;
    geometry_msgs::Vector3 volumePoseVec;
    tf::vector3TFToMsg(baseToVolume.getOrigin(), volumePoseVec);
    tf::quaternionTFToMsg(baseToVolume.getRotation(),  volumePoseQuat);

    geometry_msgs::Pose volumePose;
    volumePose.orientation = volumePoseQuat;
    volumePose.position.x = volumePoseVec.x - 0.3;
    volumePose.position.y = volumePoseVec.y - 0.3;
    volumePose.position.z = volumePoseVec.z + 0.3;

    shape_msgs::SolidPrimitive boundingBox;
    boundingBox.type = boundingBox.BOX;
    boundingBox.dimensions.resize(3);
    boundingBox.dimensions[0] = 0.6;
    boundingBox.dimensions[1] = 0.6;
    boundingBox.dimensions[2] = 0.6;

    moveit_msgs::BoundingVolume volume;
    volume.primitives.push_back(boundingBox);
    volume.primitive_poses.push_back(volumePose);

    volumeBoxPosConstraint.constraint_region = volume;
    volumeBoxPosConstraint.link_name = "sensor_constraint_frame";
    volumeBoxPosConstraint.header.frame_id = "base_link";

    cameraPoseConstraints.position_constraints.push_back(volumeBoxPosConstraint);



    // Second constraint: allow free rotation of the camera Z axis, since we mostly care about which way it's pointed around X and Y axes.

//    moveit_msgs::OrientationConstraint orientationConstraint;
//    orientationConstraint.header.frame_id = "base_link";
//    orientationConstraint.link_name = "ensenso_sensor_optical_frame";
//    orientationConstraint.orientation = targetPose.orientation;

//    orientationConstraint.absolute_z_axis_tolerance = M_PI;

//    cameraPoseConstraints.orientation_constraints.push_back(orientationConstraint);


    // Set all the constraints
    move_group.setPathConstraints(cameraPoseConstraints);


    // Make a motion planner to test poses in free space without moving the robot.
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();
    planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));

    planning_pipeline::PlanningPipelinePtr planning_pipeline(new planning_pipeline::PlanningPipeline(robot_model, node_handle_, "planning_plugin", "request_adapters"));

    planning_interface::MotionPlanRequest req;
    planning_interface::MotionPlanResponse res;

    req.group_name = "manipulator_ensenso";
    req.allowed_planning_time = 0.5;

    std::vector<double> tolerance_pose(3, 0.01);
    std::vector<double> tolerance_angle(3, 0.01);

    moveit_msgs::Constraints goal;


    // Test poses until we can find one that's reachable in constrained space.
    bool success = false;
    while(!success)
    {
      ROS_INFO_STREAM("Pose index: " << currentPoseIndex);
      if (currentPoseIndex >= move_targets.poses.size())
      {
        ROS_ERROR("Couldn't reach any of the provided poses in free space!");
        break;
      }
      // Get next best pose and try to plan a path to it in free space
      targetPose = move_targets.poses[currentPoseIndex];
      ROS_INFO_STREAM("Trying next best pose: " << targetPose);
      geometry_msgs::PoseStamped targetPoseStamped;
      targetPoseStamped.pose = targetPose;
      targetPoseStamped.header.frame_id = "base_link";
      goal = kinematic_constraints::constructGoalConstraints("ensenso_sensor_optical_frame", targetPoseStamped, tolerance_pose, tolerance_angle);
      req.goal_constraints.clear();
      req.goal_constraints.push_back(goal);
      planning_pipeline->generatePlan(planning_scene, req, res);
      if (res.error_code_.val == res.error_code_.SUCCESS)
      {
        // If it's reachable in free space, try to move to it in constrained space.
        ROS_INFO("Found a path in free space!");
        move_group.setPoseTarget(targetPose);
        if (!move_group.move())
        {
          // If not reachable in constrained space, try the next pose in free space.
          ROS_INFO("Couldn't move to this pose in constrained space");
          currentPoseIndex++;
        }
        else
        {
          // If reachable in constrained space, move to next step of NBV
          ROS_INFO("Moved to the pose in constrained space!");
          success = true;
        }
      }
      else
      {
        // If not reachable in free space, try the next pose
        ROS_ERROR("Couldn't compute a free-space trajectory");
        currentPoseIndex++;
      }
    }
  }
}



int main(int argc, char* argv[])
{
    ros::init(argc, argv, "exploration_controller_node");
    ros::NodeHandle nh;


    moveit::planning_interface::MoveGroupInterface move_group("manipulator_ensenso");
    move_group.setPlanningTime(30.0);

    Explorer explorer(nh);

    ros::AsyncSpinner async_spinner(1);
    async_spinner.start();

    explorer.MoveToNBVs(move_group);

    return 0;
}
