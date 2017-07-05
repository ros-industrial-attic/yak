/*
 * ros_sim_camera.cpp
 *
 *  Created on: Jun 13, 2017
 *      Author: jschornak
 */

#include <ros/ros_sim_camera.hpp>
#include <ros/console.h>

namespace kfusion
{

    RosSimCamera::RosSimCamera(const ros::NodeHandle& handle) :
            nodeHandle(handle), transport(handle), hasNewDepth(false), hasNewRGB(false), subscribedDepth(false), subscribedRGB(false)
    {

    }

    RosSimCamera::~RosSimCamera()
    {

    }

    kfusion::Intr RosSimCamera::GetDepthIntrinsics()
    {
        return kfusion::Intr(lastDepthInfo->K[0], lastDepthInfo->K[4], lastDepthInfo->K[2], lastDepthInfo->K[5]);
    }

    bool RosSimCamera::Grab(cv::Mat& depth, cv::Mat& image)
    {
        bool hasAny = hasNewDepth || hasNewRGB;

        if (hasNewDepth)
        {
            cv_bridge::CvImageConstPtr cvDepth = cv_bridge::toCvShare(lastDepthImage, "");
            if (!cvDepth)
            {
                ROS_ERROR("Failed to convert depth image to OpenCV");
                return false;
            }
            depth = cvDepth->image;
            hasNewDepth = false;
        }

        if (hasNewRGB)
        {
            cv_bridge::CvImageConstPtr cvColor = cv_bridge::toCvShare(lastRGBImage, "rgb8");

            if (!cvColor)
            {
                ROS_ERROR("Failed to convert color image to OpenCV");
                return false;
            }

            image = cvColor->image;
            hasNewDepth = false;
        }

        return hasAny;
    }

    void RosSimCamera::SubscribeDepth(const std::string& topic)
    {
        depthSubscriber = transport.subscribeCamera<RosRGBDCamera>(topic, 10, &RosRGBDCamera::DepthCallback, this);
        hasNewDepth = false;
        subscribedDepth = true;
    }

    void RosSimCamera::SubscribeRGB(const std::string& topic)
    {
        rgbSubscriber = transport.subscribeCamera<RosRGBDCamera>(topic, 10, &RosRGBDCamera::RGBCallback, this);
        hasNewRGB = false;
        subscribedRGB = true;
    }

    void RosSimCamera::DepthCallback(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::CameraInfoConstPtr& cameraInfo)
    {
        lastDepthImage = image;
        lastDepthInfo = cameraInfo;
        hasNewDepth = true;
    }

    void RosSimCamera::RGBCallback(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::CameraInfoConstPtr& cameraInfo)
    {
        lastRGBImage = image;
        lastRGBInfo = cameraInfo;
        hasNewRGB = true;
    }

} /* namespace kfusion */
