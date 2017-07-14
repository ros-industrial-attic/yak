//#include <ros/volume_tf_broadcaster.hpp>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>


class VolumePosePublisher {
  public:
    VolumePosePublisher(ros::NodeHandle& nh) {
//        ROS_INFO("Started constructor");
        lastPose_ = tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(0,0,0));
        subscriber_ = nh.subscribe("/volume_marker/feedback", 1000, &VolumePosePublisher::volumeTFCallback, this);
//        ROS_INFO("Finished constructor");
        VolumePosePublisher::Update();
    }

    void volumeTFCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
    {
        // Update TF transform with new volume pose
        tf::Quaternion orientation;
        tf::quaternionMsgToTF(feedback->pose.orientation, orientation);
        tf::Vector3 position;
        tf::pointMsgToTF(feedback->pose.position, position);
        lastPose_ = tf::Transform(orientation, position);

//        VolumePosePublisher::Update();

        ROS_INFO_STREAM( feedback->marker_name << " is now at " << feedback->pose.position.x << ", " << feedback->pose.position.y << ", " << feedback->pose.position.z );
    }

    void Update() {
      // Publish the most recent pose
//      ROS_INFO("Start update");
      tf::StampedTransform transformStamped(lastPose_, ros::Time::now(), "base_link", "volume_pose");

      broadcaster_.sendTransform(transformStamped);
//      ROS_INFO("update");
    }

    ros::Subscriber subscriber_;
    tf::TransformBroadcaster broadcaster_;
    tf::Transform lastPose_;
};

//void processFeedback(
//    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
//{
//  ROS_INFO_STREAM( feedback->marker_name << " is now at " << feedback->pose.position.x << ", " << feedback->pose.position.y << ", " << feedback->pose.position.z );
//}

int main(int argc, char** argv) {
  ros::init(argc, argv, "volume_tf_broadcaster");
  ros::NodeHandle nh;

  VolumePosePublisher vol(nh);
//  vol.subscriber_ = nh.subscribe("/volume_marker/feedback", 1000, &VolumePosePublisher::volumeTFCallback, &vol);

  ros::Rate rate(10.0);

  while (ros::ok()) {
    vol.Update();
    rate.sleep();
  }

//  ros::spin();
//  return 0;
}
