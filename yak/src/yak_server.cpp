#include <yak/yak_server.h>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>  // named-window apparatus; TODO: Remove this

yak::FusionServer::FusionServer(const kfusion::KinFuParams& params, const Eigen::Affine3f& world_to_volume)
  : kinfu_(new kfusion::KinFu(params))
  , volume_to_world_(world_to_volume.inverse())
  , last_camera_pose_(Eigen::Affine3f::Identity())
{
  // Debug displays
  cv::namedWindow("output", cv::WINDOW_NORMAL);
  cv::moveWindow("output", 10, 10);
  cv::resizeWindow("output", kinfu_->params().cols * 2, kinfu_->params().rows);
}

bool yak::FusionServer::fuse(const cv::Mat& depth_data, const Eigen::Affine3f& world_to_camera)
{
  // world_to_volume * world_to_camera
  // we want volume to camera
  const Eigen::Affine3f current_camera_in_volume = volume_to_world_ * world_to_camera;

  // std::cout << "Offline camera pose:\n" << current_camera_in_volume.matrix() << "\n";

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

bool yak::FusionServer::reset()
{
  kinfu_->resetVolume();
  return true;
}

bool yak::FusionServer::resetWithNewParams(const kfusion::KinFuParams& params)
{
  kinfu_.reset(new kfusion::KinFu(params));
  cv::cv2eigen(params.volume_pose.inv().matrix, volume_to_world_.matrix());
  return true;
}

// void yak::FusionServer::getCloud(pcl::PointCloud<pcl::PointXYZ>& cloud) const
//{
//  const auto points = kinfu_->downloadCloud();
//  cloud.resize(points.size());
//  std::transform(points.begin(), points.end(), cloud.begin(), [] (const kfusion::Point& pt)
//  {
//    pcl::PointXYZ pcl_pt;
//    pcl_pt.x = pt.x;
//    pcl_pt.z = pt.y;
//    pcl_pt.y = pt.z;
//    return pcl_pt;
//  });
//}

yak::TSDFContainer yak::FusionServer::downloadTSDF()
{
  const kfusion::cuda::TsdfVolume& vol = kinfu_->tsdf();
  const cv::Vec3i& vol_dims = vol.getDims();

  yak::TSDFContainer result(vol_dims[0], vol_dims[1], vol_dims[2]);
  vol.data().download(result.data());

  return result;
}

bool yak::FusionServer::step(const Eigen::Affine3f& current_pose,
                             const Eigen::Affine3f& last_pose,
                             const cv::Mat& depth)
{
  // Compute the 'step' from the last_pose to the current pose
  Eigen::Affine3f step = current_pose * last_pose.inverse();
  step = step.inverse();

  depthDevice_.upload(depth.data, depth.step, depth.rows, depth.cols);
  return kinfu_->operator()(step, current_pose, last_pose, depthDevice_);
}

void yak::FusionServer::downloadAndDisplayView()
{
  cv::Mat viewHost;
  viewHost.create(viewDevice_.rows(), viewDevice_.cols(), CV_8UC4);
  viewDevice_.download(viewHost.ptr<void>(), viewHost.step);

  cv::imshow("output", viewHost);
  cv::waitKey(1);
}

void yak::FusionServer::display()
{
  kinfu_->renderImage(viewDevice_, 3);
  downloadAndDisplayView();
}

void yak::FusionServer::display(const Eigen::Affine3f& pose)
{
  kinfu_->renderImage(viewDevice_, pose, 3);
  downloadAndDisplayView();
}
