#include "yak/offline/offline_fusion_server.h"
#include <opencv2/highgui.hpp> // named-window apparatus; TODO: Remove this

yak_offline::OfflineFusionServer::OfflineFusionServer(const kfusion::KinFuParams& params,
                                                      const Eigen::Affine3f& world_to_volume)
  : kinfu_(new kfusion::KinFu(params))
  , volume_origin_(world_to_volume)
  , last_camera_pose_(Eigen::Affine3f::Identity())
{
  // Debug displays
  cv::namedWindow("input");
  cv::moveWindow("input", 10, 1000);
  cv::namedWindow("output");
  cv::moveWindow("output", 10, 600);
}

bool yak_offline::OfflineFusionServer::fuse(const cv::Mat& depth_data, const Eigen::Affine3f& world_to_camera)
{
  const Eigen::Affine3f current_camera_in_volume = volume_origin_ * world_to_camera;

  // Compute the 'motion' from the last_pose to the current pose
  Eigen::Affine3f motion = current_camera_in_volume * last_camera_pose_.inverse();
  motion = motion.inverse();

  // Upload the depth data to the GPU for this round of fusion
  depthDevice_.upload(depth_data.data, depth_data.step, depth_data.rows, depth_data.cols);

  // Launch the fusion process
  bool result = kinfu_->operator()(motion, current_camera_in_volume, last_camera_pose_, depthDevice_);

  // Update the "last camera pose" TODO: Do I update this if the fusion step failed?
  last_camera_pose_ = current_camera_in_volume;

  display();

  return result;
}

//void yak_offline::OfflineFusionServer::fuse(const ObservationBuffer& origin_buffer)
//{
//  auto buffer = origin_buffer;
//  for (auto& pose : buffer.image_poses)
//  {
//    ROS_INFO_STREAM("\n" << pose.matrix());
//    pose = volume_origin_.cast<double>() * pose;
//    ROS_INFO_STREAM("\n" << pose.matrix());
//  }

//  for (std::size_t i = 0; i < buffer.image_data.size(); ++i)
//  {
//    const auto& depth_image = buffer.image_data[i];
//    const auto& image_pose = buffer.image_poses[i];

//    Eigen::Affine3f last_pose;
//    if (i == 0) last_pose = image_pose.cast<float>();
//    else last_pose = buffer.image_poses[i-1].cast<float>();

//    cv::imshow("input", depth_image);

//    ROS_INFO_STREAM(depth_image.channels());
//    ROS_INFO_STREAM(depth_image.elemSize());

//    cv::waitKey();

//    step(image_pose.cast<float>(), last_pose, depth_image);

//    display();
//  }
//}

void yak_offline::OfflineFusionServer::getCloud(pcl::PointCloud<pcl::PointXYZ>& cloud) const
{
  const auto points = kinfu_->downloadCloud();
  cloud.resize(points.size());
  std::transform(points.begin(), points.end(), cloud.begin(), [] (const kfusion::Point& pt)
  {
    pcl::PointXYZ pcl_pt;
    pcl_pt.x = pt.x;
    pcl_pt.z = pt.y;
    pcl_pt.y = pt.z;
    return pcl_pt;
  });
}

yak::TSDFContainer yak_offline::OfflineFusionServer::downloadTSDF()
{
  const kfusion::cuda::TsdfVolume& vol = kinfu_->tsdf();
  const cv::Vec3i& vol_dims = vol.getDims();

  yak::TSDFContainer result (vol_dims[0], vol_dims[1], vol_dims[2]);
  vol.data().download(result.data());

  return result;
}

bool yak_offline::OfflineFusionServer::step(const Eigen::Affine3f& current_pose, const Eigen::Affine3f& last_pose, const cv::Mat& depth)
{
  // Compute the 'step' from the last_pose to the current pose
  Eigen::Affine3f step = current_pose * last_pose.inverse();
  step = step.inverse();

  depthDevice_.upload(depth.data, depth.step, depth.rows, depth.cols);
  return kinfu_->operator()(step, current_pose, last_pose, depthDevice_);
}

void yak_offline::OfflineFusionServer::downloadAndDisplayView()
{
  cv::Mat viewHost;
  viewHost.create(viewDevice_.rows(), viewDevice_.cols(), CV_8UC4);
  viewDevice_.download(viewHost.ptr<void>(), viewHost.step);

  cv::imshow("output", viewHost);
  cv::waitKey(1);
}

void yak_offline::OfflineFusionServer::display()
{
  kinfu_->renderImage(viewDevice_, 3);
  downloadAndDisplayView();
}

void yak_offline::OfflineFusionServer::display(const Eigen::Affine3f& pose)
{
  kinfu_->renderImage(viewDevice_, pose, 3);
  downloadAndDisplayView();
}
