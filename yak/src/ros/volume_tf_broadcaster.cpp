//#include <ros/volume_tf_broadcaster.hpp>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <visualization_msgs/Marker.h>


class VolumePosePublisher {
  public:
    VolumePosePublisher(ros::NodeHandle& nh) {
//        ROS_INFO("Started constructor");
        lastOriginPose_ = tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(0,0,0));
        subscriber_ = nh.subscribe("/volume_marker/feedback", 1, &VolumePosePublisher::volumeTFCallback, this);

        box_line_pub_ = nh.advertise<visualization_msgs::Marker>("volume_boundary", 10);



//        ROS_INFO("Finished constructor");
        VolumePosePublisher::Update();

        int dimsX, dimsY, dimsZ;
        double res;

        nh.getParam("/kinfu/volume_dims_x", dimsX);
        nh.getParam("/kinfu/volume_dims_y", dimsY);
        nh.getParam("/kinfu/volume_dims_z", dimsZ);
        nh.getParam("/kinfu/volume_resolution", res);
//        ROS_INFO_STREAM("dimsX loaded as " << dimsX);
        sizeX_ = dimsX*res;
        sizeY_ = dimsY*res;
        sizeZ_ = dimsZ*res;


        ROS_INFO_STREAM("Volume size is " << sizeX_ << " " << sizeY_ << " " << sizeZ_);
    }

    void volumeTFCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
    {
        // Update TF transform with new volume pose
        tf::Quaternion orientation;
        tf::quaternionMsgToTF(feedback->pose.orientation, orientation);
        tf::Vector3 position;
        tf::pointMsgToTF(feedback->pose.position, position);
        lastOriginPose_ = tf::Transform(orientation, position);

//        VolumePosePublisher::Update();

        ROS_INFO_STREAM( feedback->marker_name << " is now at " << feedback->pose.position.x << ", " << feedback->pose.position.y << ", " << feedback->pose.position.z );
    }

    void Update() {
      // Publish the most recent pose
//      ROS_INFO("Start update");

      geometry_msgs::Point c1, c2, c3, c4, c5, c6, c7, c8;
      c1.x = 0;
      c1.y = 0;
      c1.z = 0;

      c2.x = -sizeX_;
      c2.y = 0;
      c2.z = 0;

      c3.x = -sizeX_;
      c3.y = -sizeY_;
      c3.z = 0;

      c4.x = 0;
      c4.y = -sizeY_;
      c4.z = 0;

      c5.x = 0;
      c5.y = 0;
      c5.z = sizeZ_;

      c6.x = -sizeX_;
      c6.y = 0;
      c6.z = sizeZ_;

      c7.x = -sizeX_;
      c7.y = -sizeY_;
      c7.z = sizeZ_;

      c8.x = 0;
      c8.y = -sizeY_;
      c8.z = sizeZ_;


      visualization_msgs::Marker box_line_list;

      box_line_list.header.frame_id =  "/volume_pose";
      box_line_list.scale.x = 0.002;
      box_line_list.action = visualization_msgs::Marker::ADD;
      box_line_list.pose.orientation.w = 1.0;
      box_line_list.type = visualization_msgs::Marker::LINE_LIST;
      box_line_list.color.r = 1.0;
      box_line_list.color.b = 1.0;
      box_line_list.color.a = 1.0;

      box_line_list.points.push_back(c1);
      box_line_list.points.push_back(c2);

      box_line_list.points.push_back(c2);
      box_line_list.points.push_back(c3);

      box_line_list.points.push_back(c3);
      box_line_list.points.push_back(c4);

      box_line_list.points.push_back(c4);
      box_line_list.points.push_back(c1);


      box_line_list.points.push_back(c5);
      box_line_list.points.push_back(c6);

      box_line_list.points.push_back(c6);
      box_line_list.points.push_back(c7);

      box_line_list.points.push_back(c7);
      box_line_list.points.push_back(c8);

      box_line_list.points.push_back(c8);
      box_line_list.points.push_back(c5);


      box_line_list.points.push_back(c1);
      box_line_list.points.push_back(c5);

      box_line_list.points.push_back(c2);
      box_line_list.points.push_back(c6);

      box_line_list.points.push_back(c3);
      box_line_list.points.push_back(c7);

      box_line_list.points.push_back(c4);
      box_line_list.points.push_back(c8);


      tf::StampedTransform transformStamped(lastOriginPose_, ros::Time::now(), "base_link", "volume_pose");

      broadcaster_.sendTransform(transformStamped);

//      box_line_pub_.publish(box_line_list);


//      ROS_INFO("update");
    }


    ros::Publisher box_line_pub_;

    double sizeX_;
    double sizeY_;
    double sizeZ_;

    ros::Subscriber subscriber_;
    tf::TransformBroadcaster broadcaster_;
    tf::Transform lastOriginPose_;
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
    ros::spinOnce();
  }

//  ros::spin();
//  return 0;
}
