#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <interactive_markers/interactive_marker_server.h>

void processFeedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  ROS_INFO_STREAM( feedback->marker_name << " is now at "
      << feedback->pose.position.x << ", " << feedback->pose.position.y
      << ", " << feedback->pose.position.z );
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "volume_tf_broadcaster");
  ros::NodeHandle nh;

  tf::TransformBroadcaster br;
  tf::Transform transform;

//  // create an interactive marker server on the topic namespace simple_marker
//  interactive_markers::InteractiveMarkerServer server("simple_marker");

//  // create an interactive marker for our server
//  visualization_msgs::InteractiveMarker int_marker;
//  int_marker.header.frame_id = "volume_link";
//  int_marker.header.stamp=ros::Time::now();
//  int_marker.name = "my_marker";
//  int_marker.description = "Simple 1-DOF Control";

//  // create a grey box marker
//  visualization_msgs::Marker box_marker;
//  box_marker.type = visualization_msgs::Marker::CUBE;
//  box_marker.scale.x = 0.45;
//  box_marker.scale.y = 0.45;
//  box_marker.scale.z = 0.45;
//  box_marker.color.r = 0.5;
//  box_marker.color.g = 0.5;
//  box_marker.color.b = 0.5;
//  box_marker.color.a = 1.0;

//  // create a non-interactive control which contains the box
//  visualization_msgs::InteractiveMarkerControl box_control;
//  box_control.always_visible = true;
//  box_control.markers.push_back( box_marker );

//  // add the control to the interactive marker
//  int_marker.controls.push_back( box_control );

//  // create a control which will move the box
//  // this control does not contain any markers,
//  // which will cause RViz to insert two arrows
//  visualization_msgs::InteractiveMarkerControl rotate_control;
//  rotate_control.name = "move_x";
//  rotate_control.interaction_mode =
//      visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;

//  // add the control to the interactive marker
//  int_marker.controls.push_back(rotate_control);

//  // add the interactive marker to our collection &
//  // tell the server to call processFeedback() when feedback arrives for it
//  server.insert(int_marker, &processFeedback);

//  // 'commit' changes and send to all clients
//  server.applyChanges();

  ros::Rate rate(10.0);

  while (nh.ok()) {
    transform.setOrigin((tf::Vector3(0.0, 0.0, 0.0)));

    transform.setRotation(tf::Quaternion(tf::Vector3(0,0,1), tfScalar(1.5708)));
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "volume_frame"));

    rate.sleep();
  }

  ros::spin();
  return 0;
}
