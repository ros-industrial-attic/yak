#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include <kfusion/kinfu.hpp>
#include <ros/ros_rgbd_camera.hpp>
#include <sensor_msgs/Image.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <ros/kinfu_server.h>
#include <interactive_markers/interactive_marker_server.h>

using namespace kfusion;

void volumeTFCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  // Need to be able to access volume pose feedback from within KinfuServer
  // publish as TF frame?
    tf::TransformBroadcaster br;

    tf::Quaternion orientation;
    tf::quaternionMsgToTF(feedback->pose.orientation, orientation);
    tf::Vector3 position;
    tf::pointMsgToTF(feedback->pose.position, position);

    tf::Transform transform(orientation, position);
    tf::StampedTransform transformStamped(transform, ros::Time::now(), "base_link", "volume_pose");

    br.sendTransform(transformStamped);

    ROS_INFO_STREAM( feedback->marker_name << " is now at " << feedback->pose.position.x << ", " << feedback->pose.position.y << ", " << feedback->pose.position.z );
}

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

    ros::Subscriber sub = node.subscribe("/volume_tf_broadcaster/feedback", 1000, volumeTFCallback);

    KinFuServer app(&camera, fixedFrame, cameraFrame);
    app.ExecuteBlocking();

//    ros::spin();

    return 0;
}
