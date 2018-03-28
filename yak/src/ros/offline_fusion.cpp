#include <ros/ros.h>
#include <opencv2/highgui.hpp>
#include <Eigen/Dense>

#include <yak/kfusion/kinfu.hpp>
#include <yak/kfusion/types.hpp>

#include <fstream>

struct ObservationBuffer
{
  std::vector<cv::Mat> image_data;
  std::vector<Eigen::Affine3d> image_poses;
};

bool loadPose(const std::string& file, Eigen::Affine3d& out)
{
  std::ifstream ifh (file.c_str());

  double x, y, z, rx, ry, rz, rw;

  ifh >> x >> y >> z >> rx >> ry >> rz >> rw;

  if (!ifh) return false;

  out.translation() = Eigen::Vector3d(x,y,z);
  Eigen::Quaterniond q;
  q.x() = rx;
  q.y() = ry;
  q.z() = rz;
  q.w() = rw;

  out.linear() = Eigen::Matrix3d(q);

  return true;
}

bool loadBuffer(const std::string& directory, ObservationBuffer& buffer)
{
  const int count = 9;

  for (int i = 1; i <= count; ++i)
  {
    const std::string img_file = directory + "depth_orig00" + std::to_string(i) + ".png";
    const std::string pose_file = directory + "pose" + std::to_string(i) + ".txt";
    cv::Mat m = cv::imread(img_file, CV_LOAD_IMAGE_ANYDEPTH);

    Eigen::Affine3d pose = Eigen::Affine3d::Identity();
    loadPose(pose_file, pose);

    buffer.image_data.push_back(m);
    buffer.image_poses.push_back(pose);
  }

  return true;
}


class OfflineFusionServer
{
public:
  OfflineFusionServer(const kfusion::KinFuParams& params)
  {
    kinfu_.reset(new kfusion::KinFu(params));
  }

  void fuse(const ObservationBuffer& buffer)
  {
    for (std::size_t i = 0; i < buffer.image_data.size(); ++i)
    {
      const auto& depth_image = buffer.image_data[i];
      const auto& image_pose = buffer.image_poses[i];

      Eigen::Affine3f last_pose;
      if (i == 0) last_pose = image_pose.cast<float>();
      else last_pose = buffer.image_poses[i-1].cast<float>();

      ROS_INFO_STREAM("Feeding image " << i << " at pose =\n" << image_pose.matrix());

      cv::imshow("input", depth_image);

      ROS_INFO_STREAM(depth_image.channels());
      ROS_INFO_STREAM(depth_image.elemSize());

      cv::waitKey();

      step(image_pose.cast<float>(), last_pose, depth_image);
    }
  }

private:
  bool step(const Eigen::Affine3f& current_pose, const Eigen::Affine3f& last_pose, const cv::Mat& depth)
  {
    // Compute the 'step' from the last_pose to the current pose
    Eigen::Affine3f step = current_pose * last_pose.inverse();

    depthDevice_.upload(depth.data, depth.step, depth.rows, depth.cols);
    return kinfu_->operator()(step, current_pose, last_pose, depthDevice_);
  }

  kfusion::KinFu::Ptr kinfu_;
  kfusion::cuda::Image viewDevice_;
  kfusion::cuda::Depth depthDevice_;
};



int main(int argc, char** argv)
{
  ros::init(argc, argv, "offline_fusion");
  ros::NodeHandle nh, pnh ("~");

  std::string buffer_dir;
  if (!pnh.getParam("buffer", buffer_dir))
  {
    ROS_ERROR("Must set 'buffer'");
    return 1;
  }

  ObservationBuffer buffer;
  if (!loadBuffer(buffer_dir, buffer))
  {
    ROS_ERROR("Failed to load buffer data");
    return 2;
  }

  // Load parameters
  kfusion::KinFuParams default_params = kfusion::KinFuParams::default_params();
//  default_params.

  OfflineFusionServer fusion (default_params);

  fusion.fuse(buffer);

  ros::spin();
  return 0;
}
