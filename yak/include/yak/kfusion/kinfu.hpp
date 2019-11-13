#ifndef KFUSION_KINFU_H_
#define KFUSION_KINFU_H_

#include "yak/kfusion/types.hpp"
#include "yak/kfusion/cuda/tsdf_volume.hpp"
#include "yak/kfusion/cuda/projective_icp.hpp"
#include <Eigen/Core>
#include <Eigen/StdVector>
#include <vector>
#include <string>

namespace kfusion
{
namespace cuda
{
KF_EXPORTS int getCudaEnabledDeviceCount();
KF_EXPORTS void setDevice(int device);
KF_EXPORTS std::string getDeviceName(int device);
KF_EXPORTS bool checkIfPreFermiGPU(int device);
KF_EXPORTS void printCudaDeviceInfo(int device);
KF_EXPORTS void printShortCudaDeviceInfo(int device);
}  // namespace cuda

struct KF_EXPORTS KinFuParams
{
  /** @brief Default values for KinFuParams. Note that some of these are almost certainly incorrect for your setup (e.g.
   * camera intrinsics)*/
  static KinFuParams default_params();

  /** @brief Number of column pixels in input images*/
  int cols;
  /** @brief Number of row pixels in input images*/
  int rows;

  /** @brief Camera intrinsic parameters [fx, fy, cx, xy]*/
  Intr intr;

  /** @brief Dimensions of the TSDF volume in voxels. Multiply by resolution to get meters*/
  Vec3i volume_dims;
  /** @brief Resolution of the TSDF volume voxels (meters/voxel)*/
  float volume_resolution;

  /** @brief Static tranform applied to the volume*/
  Affine3f volume_pose;

  /** @brief Pixels - Description TODO*/
  float bilateral_sigma_depth;
  /** @brief Pixels - Description TODO*/
  float bilateral_sigma_spatial;
  /** @brief Pixels - Description TODO*/
  int bilateral_kernel_size;

  /** @brief meters - Description TODO*/
  float icp_truncate_depth_dist;
  /** @brief meters - Description TODO*/
  float icp_dist_thres;
  /** @brief radians - Description TODO*/
  float icp_angle_thres;
  /** @brief iterations for level index 0,1,..,3*/
  std::vector<int> icp_iter_num;

  /** @brief meters - Camera must move by this amount in order for a new image to be incorporated*/
  float tsdf_min_camera_movement;
  /** @brief meters - TSDF truncation distance*/
  float tsdf_trunc_dist;
  /** @brief frames - Description TODO*/
  int tsdf_max_weight;

  /** @brief in voxel sizes - Description TODO*/
  float raycast_step_factor;
  /** @brief in voxel sizes - Description TODO*/
  float gradient_delta_factor;

  /** @brief meters - Description TODO*/
  Vec3f light_pose;

  /** @brief If true use the pose given from an external source (e.g. robot kinematics)*/
  bool use_pose_hints;
  /** @brief If true use ICP to correlate new images to the existing TSDF*/
  bool use_icp;
  /** @brief If true only apply new sensor readings when the pose has sufficiently changed from the previous reading*/
  bool update_via_sensor_motion;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class KF_EXPORTS KinFu
{
public:
  typedef cv::Ptr<KinFu> Ptr;

  KinFu(const KinFuParams& params);

  const KinFuParams& params() const;
  KinFuParams& params();

  const cuda::TsdfVolume& tsdf() const;
  cuda::TsdfVolume& tsdf();

  const cuda::ProjectiveICP& icp() const;
  cuda::ProjectiveICP& icp();

  void resetPose();
  void resetVolume();

  bool operator()(const Affine3f& inputCameraMotion,
                  const Affine3f& currentCameraPose,
                  const Affine3f& previousCameraPose,
                  const cuda::Depth& depth,
                  const cuda::Image& image = cuda::Image());

  void renderImage(cuda::Image& image, int flags = 0);
  void renderImage(cuda::Image& image, const Affine3f& pose, int flags = 0);

  std::vector<kfusion::Point> downloadCloud() const;

  Affine3f getCameraPose(int time = -1) const;

private:
  void allocate_buffers();

  int frame_counter_;
  KinFuParams params_;

  // Sensor pose, currenly calculated  via ICP
  std::vector<Affine3f, Eigen::aligned_allocator<Affine3f>> poses_;

  cuda::Dists dists_;
  cuda::Frame curr_, prev_;

  cuda::Cloud points_;
  cuda::Normals normals_;
  cuda::Depth depths_;

  cv::Ptr<cuda::TsdfVolume> volume_;
  cv::Ptr<cuda::ProjectiveICP> icp_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace kfusion
#endif
