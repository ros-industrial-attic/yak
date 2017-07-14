#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include <kfusion/kinfu.hpp>
#include <ros/ros_rgbd_camera.hpp>
#include <sensor_msgs/Image.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <ros/kinfu_server.h>

#include <interactive_markers/interactive_marker_server.h>

//#include <kinfu_node.hpp>


using namespace kfusion;

//class VolumePosePublisher {
//  public:
//    VolumePosePublisher(ros::NodeHandle& nh) {
//        lastPose_ = tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(0,0,0));
//        subscriber_ = nh.subscribe("/volume_tf_broadcaster/feedback", 1000, &VolumePosePublisher::volumeTFCallback, this);
//    }

//    void volumeTFCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
//    {
//        // Update TF transform with new volume pose
//        tf::Quaternion orientation;
//        tf::quaternionMsgToTF(feedback->pose.orientation, orientation);
//        tf::Vector3 position;
//        tf::pointMsgToTF(feedback->pose.position, position);
//        lastPose_ = tf::Transform(orientation, position);

////        VolumePosePublisher::Update();

////        ROS_INFO_STREAM( feedback->marker_name << " is now at " << feedback->pose.position.x << ", " << feedback->pose.position.y << ", " << feedback->pose.position.z );
//    }

//    void Update() {
//      // Publish the most recent pose
//      tf::StampedTransform transformStamped(lastPose_, ros::Time::now(), "base_link", "volume_pose");

//      broadcaster_.sendTransform(transformStamped);
//    }

//    ros::Subscriber subscriber_;
//    tf::TransformBroadcaster broadcaster_;
//    tf::Transform lastPose_;
//};


//    VolumePosePublisher::VolumePosePublisher(ros::NodeHandle& nh) {
//        lastPose_ = tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(0,0,0));
//        subscriber_ = nh.subscribe<visualization_msgs::InteractiveMarkerFeedbackConstPtr>("/volume_tf_broadcaster/feedback", 1000, &VolumePosePublisher::volumeTFCallback);
//    }

//    void  VolumePosePublisher::volumeTFCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
//    {
//        // Update TF transform with new volume pose
//        tf::Quaternion orientation;
//        tf::quaternionMsgToTF(feedback->pose.orientation, orientation);
//        tf::Vector3 position;
//        tf::pointMsgToTF(feedback->pose.position, position);
//        lastPose_ = tf::Transform(orientation, position);

//        ROS_INFO_STREAM( feedback->marker_name << " is now at " << feedback->pose.position.x << ", " << feedback->pose.position.y << ", " << feedback->pose.position.z );
//    }

//    void  VolumePosePublisher::Update() {
//      // Publish the most recent pose
//      tf::StampedTransform transformStamped(lastPose_, ros::Time::now(), "base_link", "volume_pose");

//      broadcaster_.sendTransform(transformStamped);
//    }



int main(int argc, char* argv[])
{
    ROS_INFO("starting kinfu node...");
    int device = 0;
    cuda::setDevice(device);
    cuda::printShortCudaDeviceInfo(device);

    if (cuda::checkIfPreFermiGPU(device))
        return std::cout << std::endl << "Kinfu is not supported for pre-Fermi GPU architectures, and not built for them by default. Exiting..." << std::endl, 1;

    ros::init(argc, argv, "yak");

    ros::NodeHandle node("~");
    RosRGBDCamera camera(node);
    camera.SubscribeDepth("/camera/depth/image_raw");
    camera.SubscribeRGB("/camera/rgb/image_rect_color");
    std::string fixedFrame = "/map";
    std::string cameraFrame = "/camera_depth_optical_frame";
    //TODO: Setting the fixed and camera frames from here doesn't seem to do anything right now. Probably would be robust to pass in the names of the actual volume nad sensor frames...
    node.param<std::string>("fixed_Frame", fixedFrame, "/map");
    node.param<std::string>("camera_frame", cameraFrame, "/camera_depth_optical_frame");

//    VolumePosePublisher vol(node);
//    vol.subscriber_ = node.subscribe<visualization_msgs::InteractiveMarkerFeedbackConstPtr>("/volume_tf_broadcaster/feedback", 1000, &VolumePosePublisher::volumeTFCallback);


//    ros::Subscriber subscriber = node.subscribe<visualization_msgs::InteractiveMarkerFeedbackConstPtr>("/volume_tf_broadcaster/feedback", 1000, &vol.volumeTFCallback);

    KinFuServer app(&camera, fixedFrame, cameraFrame);
    app.ExecuteBlocking();

//    ros::Rate rate(10.);
//    while(ros::ok())
//    {
//        vol.Update();
//        rate.sleep();
//    }

    return 0;
}
