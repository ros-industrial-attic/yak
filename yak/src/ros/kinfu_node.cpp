#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include <yak/kfusion/kinfu.hpp>
#include <yak/ros/ros_rgbd_camera.hpp>
#include <sensor_msgs/Image.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <yak/ros/kinfu_server.h>

#include <interactive_markers/interactive_marker_server.h>

using namespace kfusion;

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
    std::string fixedFrame;
    std::string cameraFrame;

    node.param<std::string>("fixed_frame", fixedFrame, "/map");
    node.param<std::string>("camera_frame", cameraFrame, "/camera_depth_optical_frame");
    ROS_INFO_STREAM("Fixed frame: " + fixedFrame + " Camera frame: " + cameraFrame);

    KinFuServer app(&camera, fixedFrame, cameraFrame);
    app.ExecuteBlocking();

    return 0;
}
