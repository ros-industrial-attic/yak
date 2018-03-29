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

std::string toFormattedInt(int i)
{
  if (i < 10) return "00" + std::to_string(i);
  else if (i < 100) return "0" + std::to_string(i);
  else return std::to_string(i);
}

bool loadBuffer(const std::string& directory, ObservationBuffer& buffer)
{
  const int count = 362;

  for (int i = 1; i <= count; ++i)
  {
    const std::string img_file = directory + "depth_orig" + toFormattedInt(i) + ".png";
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
  OfflineFusionServer(const kfusion::KinFuParams& params, const Eigen::Affine3f& world_to_volume)
  {
    kinfu_.reset(new kfusion::KinFu(params));

    // Debug displays
    cv::namedWindow("input");
    cv::moveWindow("input", 10, 1000);
    cv::namedWindow("output");
    cv::moveWindow("output", 10, 600);

    volume_origin_ = world_to_volume;
  }

  void fuse(const ObservationBuffer& origin_buffer)
  {
    auto buffer = origin_buffer;
    for (auto& pose : buffer.image_poses)
    {
      ROS_INFO_STREAM("\n" << pose.matrix());
      pose = volume_origin_.cast<double>() * pose;
      ROS_INFO_STREAM("\n" << pose.matrix());
    }

    for (std::size_t i = 0; i < buffer.image_data.size(); ++i)
    {
      const auto& depth_image = buffer.image_data[i];
      const auto& image_pose = buffer.image_poses[i];

      Eigen::Affine3f last_pose;
      if (i == 0) last_pose = image_pose.cast<float>();
      else last_pose = buffer.image_poses[i-1].cast<float>();

      cv::imshow("input", depth_image);

      ROS_INFO_STREAM(depth_image.channels());
      ROS_INFO_STREAM(depth_image.elemSize());

      cv::waitKey();

      step(image_pose.cast<float>(), last_pose, depth_image);

      display();
    }
  }

private:
  bool step(const Eigen::Affine3f& current_pose, const Eigen::Affine3f& last_pose, const cv::Mat& depth)
  {
    // Compute the 'step' from the last_pose to the current pose
    Eigen::Affine3f step = current_pose * last_pose.inverse();
    step = step.inverse();

    depthDevice_.upload(depth.data, depth.step, depth.rows, depth.cols);
    return kinfu_->operator()(step, current_pose, last_pose, depthDevice_);
  }

  void display()
  {
    kinfu_->renderImage(viewDevice_, 3);

    cv::Mat viewHost;
    viewHost.create(viewDevice_.rows(), viewDevice_.cols(), CV_8UC4);
    viewDevice_.download(viewHost.ptr<void>(), viewHost.step);

    cv::imshow("output", viewHost);
  }

  kfusion::KinFu::Ptr kinfu_;
  kfusion::cuda::Image viewDevice_;
  kfusion::cuda::Depth depthDevice_;
  Eigen::Affine3f volume_origin_;
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
  default_params.use_icp = false;
  default_params.use_pose_hints = true;
  default_params.update_via_sensor_motion = false;

  default_params.volume_dims = cv::Vec3i(1024, 1024, 1024);
  default_params.volume_resolution = 0.005;

  // World to volume
  Eigen::Affine3f world_to_volume (Eigen::Affine3f::Identity());

  world_to_volume.translation() = Eigen::Vector3f(default_params.volume_dims[0] * default_params.volume_resolution / 2.0,
                                                  default_params.volume_dims[1] * default_params.volume_resolution / 2.0,
                                                  -0.01);//default_params.volume_dims[2] * default_params.volume_resolution / 2.0);

  default_params.volume_pose = world_to_volume * buffer.image_poses.front().cast<float>();

  default_params.tsdf_trunc_dist = 0.04f; //meters;
  default_params.tsdf_max_weight = 5;   //frames
  default_params.raycast_step_factor = 0.25;  //in voxel sizes
  default_params.gradient_delta_factor = 0.25; //in voxel sizes



  OfflineFusionServer fusion (default_params, world_to_volume);

  fusion.fuse(buffer);

  ros::spin();
  return 0;
}
