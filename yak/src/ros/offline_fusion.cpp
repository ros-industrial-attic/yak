#include <ros/ros.h>
#include <opencv2/highgui.hpp>
#include <Eigen/Dense>

#include <yak/kfusion/kinfu.hpp>
#include <yak/kfusion/types.hpp>

#include <yak/ros/offline_fusion.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <yak/ros/half.hpp>

#include <fstream>

#include <yak/mc/marching_cubes.h>
#include <pcl/io/ply_io.h>

struct ObservationBuffer
{
  std::vector<cv::Mat> image_data;
  std::vector<Eigen::Affine3d> image_poses;
};

bool loadPose(const std::string& file, Eigen::Affine3d& out)
{
  std::ifstream ifh (file.c_str());

  if (!ifh.is_open()) return false;

  double x, y, z, rx, ry, rz, rw;

  ifh >> x >> y >> z >> rx >> ry >> rz >> rw;

  out.translation() = Eigen::Vector3d(x,y,z);
  Eigen::Quaterniond q;
  q.x() = rx;
  q.y() = ry;
  q.z() = rz;
  q.w() = rw;

  out.linear() = Eigen::Matrix3d(q);

  return static_cast<bool>(ifh);
}

std::string toFormattedInt(int i)
{
  if (i < 10) return "00" + std::to_string(i);
  else if (i < 100) return "0" + std::to_string(i);
  else return std::to_string(i);
}

bool loadBuffer(const std::string& directory, ObservationBuffer& buffer)
{
  unsigned counter = 0;

  while (true)
  {
    const std::string img_file = directory + "depth_" + std::to_string(counter) + ".png";
    const std::string pose_file = directory + "pose_" + std::to_string(counter) + ".txt";
    Eigen::Affine3d pose = Eigen::Affine3d::Identity();

    if (!loadPose(pose_file, pose))
    {
      ROS_INFO("Done loading %d", counter);
      break;
    }

    cv::Mat m = cv::imread(img_file, CV_LOAD_IMAGE_ANYDEPTH);

    buffer.image_data.push_back(m);
    buffer.image_poses.push_back(pose);

    counter += 1;
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

  void getCloud(pcl::PointCloud<pcl::PointXYZ>& cloud) const
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

  TSDFContainer downloadTSDF()
  {
    const kfusion::cuda::TsdfVolume& vol = kinfu_->tsdf();

    TSDFContainer result (vol.getDims()[0], vol.getDims()[1], vol.getDims()[2]);
    vol.data().download(result.data());

    return result;
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
public:
  void display(const Eigen::Affine3f& pose)
  {
    kinfu_->renderImage(viewDevice_, pose, 3);
    cv::Mat viewHost;
    viewHost.create(viewDevice_.rows(), viewDevice_.cols(), CV_8UC4);
    viewDevice_.download(viewHost.ptr<void>(), viewHost.step);

    cv::imshow("output", viewHost);
  }
private:
  kfusion::KinFu::Ptr kinfu_;
  kfusion::cuda::Image viewDevice_;
  kfusion::cuda::Depth depthDevice_;
  Eigen::Affine3f volume_origin_;
};


void flythrough(OfflineFusionServer& server, const Eigen::Affine3f& start_pose, const Eigen::Affine3f& world_to_vol)
{
  Eigen::Affine3f active_pose = world_to_vol * start_pose;
  int key = 0;

  const double step_size = 0.03;

  ROS_INFO("Start");
  while (key != 'q')
  {
    ROS_INFO("loop");
    server.display(active_pose);
    switch (key)
    {
    case 65361: active_pose *= Eigen::Translation3f(-step_size, 0, 0); break;
    case 65363: active_pose *= Eigen::Translation3f(step_size, 0, 0); break;
    case 65362: active_pose *= Eigen::Translation3f(0.0, 0.0, step_size); break;
    case 65364: active_pose *= Eigen::Translation3f(0, 0, -step_size); break;
    case 'a': active_pose *= Eigen::AngleAxisf(-0.1, Eigen::Vector3f::UnitY()); break;
    case 'd': active_pose *= Eigen::AngleAxisf(0.1, Eigen::Vector3f::UnitY()); break;
    case 'w': active_pose *= Eigen::Translation3f(0, -step_size, 0); break;
    case 's': active_pose *= Eigen::Translation3f(0, step_size, 0); break;

    }
    ROS_INFO("Wait");
    key = cv::waitKeyEx(0);
    ROS_INFO_STREAM(key);
  }
}
//case 28: // right arrow

//case 29: // left arrow

//case 30: // up arrow

//case 31: // down arrow


pcl::PointCloud<pcl::PointXYZ> naiveSurfaceCloud(const TSDFContainer& tsdf, float res)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;

  const int min_weight = 1;
  const float min_thresh = (0.006 * 1) / (0.006 * 5.0);//0.75f;

  for (int i = 0; i < tsdf.size(); ++i)
  {
    half_float::half dist;
    uint16_t weight;
    tsdf.read(i, dist, weight);

    if (weight > min_weight && std::abs(dist) < min_thresh) {

      int x,y,z;
      tsdf.fromIndex(i, x, y, z);

      pcl::PointXYZ pt;
      pt.x = x * res;
      pt.y = y * res;
      pt.z = z * res;

      cloud.push_back(pt);
    }

  }

  return cloud;
}

pcl::PointCloud<pcl::PointXYZ> unseenCloud(const TSDFContainer& tsdf, float res)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;

  const int max_weight = 2;

  for (int i = 0; i < tsdf.size(); ++i)
  {
    half_float::half dist;
    uint16_t weight;
    tsdf.read(i, dist, weight);

    if (weight < max_weight) {

      int x,y,z;
      tsdf.fromIndex(i, x, y, z);

      pcl::PointXYZ pt;
      pt.x = x * res;
      pt.y = y * res;
      pt.z = z * res;

      cloud.push_back(pt);
    }

  }

  return cloud;
}

enum class VoxelState {
  Unseen, Empty, Near
};

VoxelState classify(half_float::half dist, uint16_t weight)
{
  const int unseen_weight = 1;
  const float truncation_dist = 1.0f;

  if (weight < unseen_weight) return VoxelState::Unseen;
  else if (std::abs(dist) < truncation_dist) return VoxelState::Near;
  else return VoxelState::Empty;
}

pcl::PointCloud<pcl::PointXYZ> unseenEmptyInterface(const TSDFContainer& tsdf, float res)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;

  auto check_empty = [&tsdf] (int x, int y, int z) {
    if (x < 0 || y < 0 || z < 0) return false;
    if (x >= tsdf.dims()[0] || y >= tsdf.dims()[1] || z >= tsdf.dims()[2]) return false;

    half_float::half h;
    uint16_t w;

    tsdf.read(tsdf.toIndex(x,y,z), h, w);

    return classify(h, w) == VoxelState::Empty;
  };

  for (int i = 0; i < tsdf.size(); ++i)
  {
    // Read current cell
    half_float::half dist;
    uint16_t weight;
    tsdf.read(i, dist, weight);

    // If the current cell is un-observed
    if (classify(dist, weight) == VoxelState::Unseen)
    {
      int x,y,z;
      tsdf.fromIndex(i, x, y, z);

      // Check the four cardinal neighbors
      if (check_empty(x + 1, y, z) || check_empty(x - 1, y, z)
          || check_empty(x, y - 1, z) || check_empty(x, y + 1, z)
          || check_empty(x, y, z - 1) || check_empty(x, y, z + 1))
      {
        pcl::PointXYZ pt;
        pt.x = x * res;
        pt.y = y * res;
        pt.z = z * res;

        cloud.push_back(pt);
      }
    }
  }

  return cloud;
}

void printStats(const TSDFContainer& tsdf)
{
  double min_dist = 1e6, max_dist = -1e6, avg_dist = 0;
  double min_weight = 1e6, max_weight = -1e6, avg_weight = 0;

  for (int i = 0; i < tsdf.size(); ++i)
  {
    // Read current cell
    half_float::half dist;
    uint16_t weight;
    tsdf.read(i, dist, weight);

    double d = dist;
    double w = weight;

    min_dist = std::min(min_dist, d);
    min_weight = std::min(min_weight, w);

    max_dist = std::max(max_dist, d);
    max_weight = std::max(max_weight, w);

    avg_dist += d;
    avg_weight += w;
  }

  ROS_INFO("Min/Max Dist: %f \t %f", min_dist, max_dist);
  ROS_INFO("Avg dist: %f", avg_dist / tsdf.size());
  ROS_INFO("Min/Max Weight: %f \t %f", min_weight, max_weight);
  ROS_INFO("Avg weight: %f", avg_weight / tsdf.size());
}

pcl::PointCloud<pcl::PointXYZ> projectAll(const kfusion::Intr& intr,
                                          const ObservationBuffer& buffer)
{
  pcl::PointCloud<pcl::PointXYZ> result;

  // For each image
  for (std::size_t i = 0; i < buffer.image_data.size(); ++i)
  {
    const auto& img_pose = buffer.image_poses[i];
    const auto& img = buffer.image_data[i];

    // For each pixel
    for (auto j = 0; j < img.rows; ++j)
    {
      for (auto k = 0; k < img.cols; ++k)
      {
        uint16_t depth = img.at<uint16_t>(j, k);

        if (depth == 0) continue;
        // Project the pixel into the camera space
        Eigen::Vector3d pt_in_cam;
        pt_in_cam.z() = static_cast<double>(depth) / 1000.0;
        pt_in_cam.x() = (k - intr.cx) * pt_in_cam.z() / intr.fx;
        pt_in_cam.y() = (j - intr.cy) * pt_in_cam.z() / intr.fy;

        // Transform the point into world space
        Eigen::Vector3d pt_in_world = img_pose * pt_in_cam;

        // Add to results
        pcl::PointXYZ pcl_pt;
        pcl_pt.x = pt_in_world.x();
        pcl_pt.y = pt_in_world.y();
        pcl_pt.z = pt_in_world.z();
        result.push_back(pcl_pt);
      }
    }
  }

  return result;
}

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

  default_params.volume_dims = cv::Vec3i(1248, 512, 512);
  default_params.volume_resolution = 0.006;

  // World to volume
  Eigen::Affine3f world_to_volume (Eigen::Affine3f::Identity());

  world_to_volume.translation() = Eigen::Vector3f(default_params.volume_dims[0] * default_params.volume_resolution / 2.0,
                                                  default_params.volume_dims[1] * default_params.volume_resolution / 2.0,
                                                  -0.1);//default_params.volume_dims[2] * default_params.volume_resolution / 2.0);

  default_params.volume_pose = world_to_volume * buffer.image_poses.front().cast<float>();

  default_params.tsdf_trunc_dist = default_params.volume_resolution * 5.0; //meters;
  default_params.tsdf_max_weight = 50;   //frames
  default_params.raycast_step_factor = 0.25;  //in voxel sizes
  default_params.gradient_delta_factor = 0.25; //in voxel sizes



  OfflineFusionServer fusion (default_params, world_to_volume);

  fusion.fuse(buffer);

  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.header.frame_id = "world";
  fusion.getCloud(cloud);

  ros::Publisher pub = nh.advertise<decltype(cloud)>("cloud", 1, true);

  ros::Rate pub_rate (1.0);
  ROS_INFO_STREAM("publishing cloud w/ n-points = " << cloud.size());

  pub.publish(cloud);

  ros::spinOnce();

  flythrough(fusion, buffer.image_poses.back().cast<float>(), world_to_volume);

  // Download cloud
  auto tsdf = fusion.downloadTSDF();

  ROS_INFO_STREAM("Downloaded");
  ROS_INFO_STREAM("Size: " << tsdf.size());
  ROS_INFO_STREAM("DIMS: " << tsdf.dims().transpose());

  printStats(tsdf);

  // naive surface
  ros::Publisher pub_naive = nh.advertise<decltype(cloud)>("cloud_naive", 1, true);
  auto naive_cloud = naiveSurfaceCloud(tsdf, default_params.volume_resolution);
  naive_cloud.header.frame_id = "world";
  pub_naive.publish(naive_cloud);
  pcl::io::savePCDFileBinary("surface.pcd", naive_cloud);
  ROS_INFO_STREAM("naive");

  // unseen
//  ros::Publisher pub_unseen = nh.advertise<decltype(cloud)>("cloud_unseen", 1, true);
//  auto unseen_cloud = unseenCloud(tsdf, default_params.volume_resolution);
//  unseen_cloud.header.frame_id = "world";
//  pub_unseen.publish(unseen_cloud);
//  pcl::io::savePCDFileBinary("unseen.pcd", unseen_cloud);
//  ROS_INFO_STREAM("unseen: " << unseen_cloud.size());

  // interface
  ros::Publisher pub_interface = nh.advertise<decltype(cloud)>("cloud_interface", 1, true);
  auto interface_cloud = unseenEmptyInterface(tsdf, default_params.volume_resolution);
  interface_cloud.header.frame_id = "world";
  pub_interface.publish(interface_cloud);
  pcl::io::savePCDFileBinary("interface.pcd", interface_cloud);
  ROS_INFO_STREAM("interface: " << interface_cloud.size());

  ROS_INFO_STREAM("Starting marching cubes...");
  auto mesh = yak::marchingCubesCPU(tsdf);
  pcl::io::savePLYFileBinary("cubes.ply", mesh);

  ROS_INFO_STREAM("All done!");

//  auto raw_cloud = projectAll(default_params.intr, buffer);
//  pcl::io::savePCDFileBinary("raw.pcd", raw_cloud);
//  ROS_INFO_STREAM("raw");

  ros::spin();

  return 0;
}
