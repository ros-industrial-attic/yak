#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

void processFeedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  ROS_INFO_STREAM( feedback->marker_name << " is now at " << feedback->pose.position.x << ", " << feedback->pose.position.y << ", " << feedback->pose.position.z );
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "volume_marker");
  ros::NodeHandle nh;

//  tf::TransformBroadcaster br;
//  tf::Transform transform;

  // create an interactive marker server on the topic namespace simple_marker
  interactive_markers::InteractiveMarkerServer server("volume_marker");

  // create an interactive marker for our server
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header.frame_id = "base_link";
  int_marker.header.stamp=ros::Time::now();
  int_marker.name = "my_marker";
  int_marker.description = "Voxel Volume Pose";

  // create a grey box marker
  visualization_msgs::Marker box_marker;
  box_marker.type = visualization_msgs::Marker::CUBE;
  box_marker.scale.x = 0.45;
  box_marker.scale.y = 0.45;
  box_marker.scale.z = 0.45;
  box_marker.color.r = 0.5;
  box_marker.color.g = 0.5;
  box_marker.color.b = 0.5;
  box_marker.color.a = 1.0;

  // create a non-interactive control which contains the box
  visualization_msgs::InteractiveMarkerControl box_control;
  box_control.always_visible = true;
  box_control.markers.push_back( box_marker );

  // add the control to the interactive marker
  int_marker.controls.push_back( box_control );

  // create a control which will move the box
  // this control does not contain any markers,
  // which will cause RViz to insert two arrows
  visualization_msgs::InteractiveMarkerControl rotate_control;
//  rotate_control.name = "move_x";
//  rotate_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
//  rotate_control.orientation_mode = visualization_msgs::InteractiveMarkerControl::FIXED;

  rotate_control.orientation.w = 1;
  rotate_control.orientation.x = 1;
  rotate_control.orientation.y = 0;
  rotate_control.orientation.z = 0;
//  rotate_control.name = "rotate_x";
//  rotate_control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
//  int_marker.controls.push_back(control);
  rotate_control.name = "move_x";
  rotate_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  rotate_control.orientation_mode = visualization_msgs::InteractiveMarkerControl::FIXED;
  int_marker.controls.push_back(rotate_control);

  rotate_control.orientation.w = 1;
  rotate_control.orientation.x = 0;
  rotate_control.orientation.y = 1;
  rotate_control.orientation.z = 0;
//  rotate_control.name = "rotate_z";
//  rotate_control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
//  int_marker.controls.push_back(control);
  rotate_control.name = "move_z";
  rotate_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  rotate_control.orientation_mode = visualization_msgs::InteractiveMarkerControl::FIXED;
  int_marker.controls.push_back(rotate_control);

  rotate_control.orientation.w = 1;
  rotate_control.orientation.x = 0;
  rotate_control.orientation.y = 0;
  rotate_control.orientation.z = 1;
//  rotate_control.name = "rotate_y";
//  rotate_control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
//  int_marker.controls.push_back(control);
  rotate_control.name = "move_y";
  rotate_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  rotate_control.orientation_mode = visualization_msgs::InteractiveMarkerControl::FIXED;
  int_marker.controls.push_back(rotate_control);

  // add the control to the interactive marker
  int_marker.controls.push_back(rotate_control);

  // add the interactive marker to our collection &
  // tell the server to call processFeedback() when feedback arrives for it
  server.insert(int_marker, &processFeedback);

  // 'commit' changes and send to all clients
  server.applyChanges();

//  VolumePosePublisher vol(nh);

//  ros::Rate rate(10.0);

//  while (ros::ok()) {
////    vol.Update();
//    rate.sleep();
//  }

  ros::spin();
//  return 0;
}
