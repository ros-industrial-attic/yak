#include "precomp.hpp"
#include "internal.hpp"
#include <stdio.h>
#include <ros/ros.h>
using namespace std;
using namespace kfusion;
using namespace kfusion::cuda;

static inline float deg2rad(float alpha)
{
    return alpha * 0.017453293f;
}

kfusion::KinFuParams kfusion::KinFuParams::default_params()
{
    const int iters[] = { 10, 5, 4, 0 };
    const int levels = sizeof(iters) / sizeof(iters[0]);

    KinFuParams p;

    p.cols = 640;  //pixels
    p.rows = 480;  //pixels
    p.intr = Intr(525.f, 525.f, p.cols / 2 - 0.5f, p.rows / 2 - 0.5f);

    p.volume_dims = Vec3i::all(512);  //number of voxels
//    p.volume_size = Vec3f::all(3.f);  //meters
    p.volume_resolution = 0.005859375;



//    p.volume_pose = Affine3f().translate(Vec3f(-p.volume_size[0] / 2, -p.volume_size[1] / 2, 0.5f));

    p.volume_pose = Affine3f::Identity().translate(Vec3f(1,0,0));

    p.bilateral_sigma_depth = 0.04f;  //meter
    p.bilateral_sigma_spatial = 4.5; //pixels
    p.bilateral_kernel_size = 7;     //pixels

    p.icp_truncate_depth_dist = 0.f;        //meters, disabled
    p.icp_dist_thres = 0.1f;                //meters
    p.icp_angle_thres = deg2rad(30.f); //radians
    p.icp_iter_num.assign(iters, iters + levels);

    p.tsdf_min_camera_movement = 0.f; //meters, disabled
    p.tsdf_trunc_dist = 0.04f; //meters;
    p.tsdf_max_weight = 64;   //frames

    p.raycast_step_factor = 0.75f;  //in voxel sizes
    p.gradient_delta_factor = 0.5f; //in voxel sizes

    //p.light_pose = p.volume_pose.translation()/4; //meters
    p.light_pose = Vec3f::all(0.f); //meters

    p.use_pose_hints = false;
    p.use_icp = true;
    p.update_via_sensor_motion = true;

    return p;
}

kfusion::KinFu::KinFu(const KinFuParams& params) :
        frame_counter_(0), params_(params)
{
    CV_Assert(params.volume_dims[0] % 32 == 0);

    volume_ = cv::Ptr<cuda::TsdfVolume>(new cuda::TsdfVolume(params_.volume_dims));

    volume_->setTruncDist(params_.tsdf_trunc_dist);
    volume_->setMaxWeight(params_.tsdf_max_weight);

    // Set the metric dimensions of the volume using the voxel dimensions and the metric voxel resolution
    Vec3f volumeSize(params_.volume_dims[0]*params_.volume_resolution, params_.volume_dims[1]*params_.volume_resolution, params_.volume_dims[2]*params_.volume_resolution);
    volume_->setSize(volumeSize);
    ROS_INFO_STREAM("Volume size set to: " << volume_->getSize());
//    volume_->setPose(params_.volume_pose);
    volume_->setPose(Affine3f::Identity());
    ROS_INFO_STREAM("Volume pose set to: " << volume_->getPose().matrix);
    volume_->setRaycastStepFactor(params_.raycast_step_factor);
    volume_->setGradientDeltaFactor(params_.gradient_delta_factor);

     // TODO: Modify ICP to optionally use TF pose as hint for camera pose calculation.
    icp_ = cv::Ptr<cuda::ProjectiveICP>(new cuda::ProjectiveICP());
    icp_->setDistThreshold(params_.icp_dist_thres);
    icp_->setAngleThreshold(params_.icp_angle_thres);
    icp_->setIterationsNum(params_.icp_iter_num);

    allocate_buffers();
    resetVolume();
    // Need to reserve poses on start, else it crashes.
    poses_.reserve(30000);

    // TODO: Allow loading of robot pose instead of default volume pose
    poses_.push_back(params_.volume_pose.matrix);
}

const kfusion::KinFuParams& kfusion::KinFu::params() const
{
    return params_;
}

kfusion::KinFuParams& kfusion::KinFu::params()
{
    return params_;
}

const kfusion::cuda::TsdfVolume& kfusion::KinFu::tsdf() const
{
    return *volume_;
}

kfusion::cuda::TsdfVolume& kfusion::KinFu::tsdf()
{
    return *volume_;
}

const kfusion::cuda::ProjectiveICP& kfusion::KinFu::icp() const
{
    return *icp_;
}

kfusion::cuda::ProjectiveICP& kfusion::KinFu::icp()
{
    return *icp_;
}

void kfusion::KinFu::allocate_buffers()
{
    const int LEVELS = cuda::ProjectiveICP::MAX_PYRAMID_LEVELS;

    int cols = params_.cols;
    int rows = params_.rows;

    dists_.create(rows, cols);

    curr_.depth_pyr.resize(LEVELS);
    curr_.normals_pyr.resize(LEVELS);
    prev_.depth_pyr.resize(LEVELS);
    prev_.normals_pyr.resize(LEVELS);

    curr_.points_pyr.resize(LEVELS);
    prev_.points_pyr.resize(LEVELS);

    for (int i = 0; i < LEVELS; ++i)
    {
        curr_.depth_pyr[i].create(rows, cols);
        curr_.normals_pyr[i].create(rows, cols);

        prev_.depth_pyr[i].create(rows, cols);
        prev_.normals_pyr[i].create(rows, cols);

        curr_.points_pyr[i].create(rows, cols);
        prev_.points_pyr[i].create(rows, cols);

        cols /= 2;
        rows /= 2;
    }

    depths_.create(params_.rows, params_.cols);
    normals_.create(params_.rows, params_.cols);
    points_.create(params_.rows, params_.cols);
}

void kfusion::KinFu::resetPose()
{

    cout << "Reset Pose" << endl;

//    frame_counter_ = 0;

    // TODO: Don't reset to initially-specified camera-relative pose if using a volume pose defined in a global context
    poses_.clear();
    poses_.reserve(30000);

    // TODO: Allow loading of robot pose instead of default volume pose
    poses_.push_back(params_.volume_pose.matrix);
    cout << "Resetting to: " << params_.volume_pose.matrix << endl;

//    volume_->clear();
}

void kfusion::KinFu::resetVolume()
{
    frame_counter_ = 0;
    cout << "Reset Volume" << endl;
    volume_->clear();
}

kfusion::Affine3f kfusion::KinFu::getCameraPose(int time) const
{
    if (time > (int) poses_.size() || time < 0)
        time = (int) poses_.size() - 1;
    return poses_[time];
}

bool kfusion::KinFu::operator()(const Affine3f& inputCameraMotion, const Affine3f& cameraPose, const kfusion::cuda::Depth& depth, const kfusion::cuda::Image& /*image*/)
{
    const KinFuParams& p = params_;
    const int LEVELS = icp_->getUsedLevelsNum();

    cuda::computeDists(depth, dists_, p.intr);
    cuda::depthBilateralFilter(depth, curr_.depth_pyr[0], p.bilateral_kernel_size, p.bilateral_sigma_spatial, p.bilateral_sigma_depth);

    if (p.icp_truncate_depth_dist > 0)
        kfusion::cuda::depthTruncation(curr_.depth_pyr[0], p.icp_truncate_depth_dist);

    for (int i = 1; i < LEVELS; ++i)
        cuda::depthBuildPyramid(curr_.depth_pyr[i - 1], curr_.depth_pyr[i], p.bilateral_sigma_depth);

    for (int i = 0; i < LEVELS; ++i)
#if defined USE_DEPTH
        cuda::computeNormalsAndMaskDepth(p.intr, curr_.depth_pyr[i], curr_.normals_pyr[i]);
#else
        cuda::computePointNormals(p.intr(i), curr_.depth_pyr[i], curr_.points_pyr[i], curr_.normals_pyr[i]);
#endif

    cuda::waitAllDefaultStream();

    //can't perform more on first frame
    if (frame_counter_ == 0)
    {
        volume_->integrate(dists_, poses_.at(poses_.size() - 1), p.intr);
#if defined USE_DEPTH
        curr_.depth_pyr.swap(prev_.depth_pyr);
#else
        curr_.points_pyr.swap(prev_.points_pyr);
#endif
        curr_.normals_pyr.swap(prev_.normals_pyr);
        return ++frame_counter_, false;
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // ICP

    // ICP compares the points and normals between the current and previous depth images to estimate the affine transform from the previous image pose to the current image pose.
    // If it finds a transform with error below the threshold, the previous pose is multiplied by the affine transform to produce the new pose.
    // If no transform is found (for one reason or another) a reset is triggered.


    // DONE: Make TF listener. Calculate affine transform between last pose and the new pose from TF. Pass this into estimateTransform and don't overwrite it with an identity matrix.

    Affine3f cameraMotion = Affine3f::Identity(); // cuur -> prev
    Affine3f cameraMotionCorrected;
    Affine3f cameraPoseCorrected = cameraPose;
    if (params_.use_pose_hints) {
      cameraMotion = inputCameraMotion;
    }

    {
        //ScopeTime time("icp");
#if defined USE_DEPTH
        bool ok = icp_->estimateTransform(affine, p.intr, curr_.depth_pyr, curr_.normals_pyr, prev_.depth_pyr, prev_.normals_pyr);
#else
    bool ok = true;
    if (params_.use_icp) {
        ok =  icp_->estimateTransform(cameraMotion, cameraMotionCorrected, p.intr, curr_.points_pyr, curr_.normals_pyr, prev_.points_pyr, prev_.normals_pyr);
//        cameraPoseCorrected = cameraPose * cameraMotion.inv() * cameraMotionCorrected;
        cameraPoseCorrected = cameraPose;
//        cameraMotion = cameraMotionCorrected;
    }
    else {
      cameraPoseCorrected = cameraPose;
      cameraMotionCorrected = cameraMotion;
    }

#endif
        if (!ok)
            return resetVolume(), resetPose(), false;
    }

//    poses_.push_back(poses_.back() * cameraMotion);
    if (params_.use_icp){
        // Update pose with latest measured pose
        // TODO: Make this not put the estimated sensor pose in the wrong position relative to the global frame.
        cout << "Updating via motion" << endl;
        poses_.push_back(poses_.back() * cameraMotionCorrected);
    } else {
        // Update pose estimate using latest camera motion transform
        cout << "Updating via pose" << endl;
        poses_.push_back(cameraPoseCorrected);
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Volume integration

    // This is the transform from the origin of the volume to the camera.
    cout << "Newest pose is  " << poses_.back().matrix << endl;

    // We do not integrate volume if camera does not move.
    // TODO: As it turns out I do care about this! Leaving the camera in one place introduces a lot of noise. Come up with a better metric for determining motion.
    float rnorm = (float) cv::norm(cameraMotion.rvec());
    float tnorm = (float) cv::norm(cameraMotion.translation());

    cout << "Rnorm " << rnorm << "  Tnorm " << tnorm << endl;

    bool integrate = (rnorm + tnorm) / 2 >= p.tsdf_min_camera_movement;
    if (integrate)
    {
        //ScopeTime time("tsdf");
        volume_->integrate(dists_, poses_.back(), p.intr);
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    // Ray casting
    {
        //ScopeTime time("ray-cast-all");
#if defined USE_DEPTH
        volume_->raycast(poses_.back(), p.intr, prev_.depth_pyr[0], prev_.normals_pyr[0]);
        for (int i = 1; i < LEVELS; ++i)
        resizeDepthNormals(prev_.depth_pyr[i-1], prev_.normals_pyr[i-1], prev_.depth_pyr[i], prev_.normals_pyr[i]);
#else
        volume_->raycast(poses_.back(), p.intr, prev_.points_pyr[0], prev_.normals_pyr[0]);
        for (int i = 1; i < LEVELS; ++i)
            resizePointsNormals(prev_.points_pyr[i - 1], prev_.normals_pyr[i - 1], prev_.points_pyr[i], prev_.normals_pyr[i]);
#endif
        cuda::waitAllDefaultStream();
    }

    return ++frame_counter_, true;
}

void kfusion::KinFu::renderImage(cuda::Image& image, int flag)
{
    const KinFuParams& p = params_;
    image.create(p.rows, flag != 3 ? p.cols : p.cols * 2);

#if defined USE_DEPTH
#define PASS1 prev_.depth_pyr
#else
#define PASS1 prev_.points_pyr
#endif

    if (flag < 1 || flag > 3)
        cuda::renderImage(PASS1[0], prev_.normals_pyr[0], params_.intr, params_.light_pose, image);
    else if (flag == 2)
        cuda::renderTangentColors(prev_.normals_pyr[0], image);
    else /* if (flag == 3) */
    {
        DeviceArray2D<RGB> i1(p.rows, p.cols, image.ptr(), image.step());
        DeviceArray2D<RGB> i2(p.rows, p.cols, image.ptr() + p.cols, image.step());

        cuda::renderImage(PASS1[0], prev_.normals_pyr[0], params_.intr, params_.light_pose, i1);
        cuda::renderTangentColors(prev_.normals_pyr[0], i2);
    }
#undef PASS1
}

void kfusion::KinFu::renderImage(cuda::Image& image, const Affine3f& pose, int flag)
{
    const KinFuParams& p = params_;
    image.create(p.rows, flag != 3 ? p.cols : p.cols * 2);
    depths_.create(p.rows, p.cols);
    normals_.create(p.rows, p.cols);
    points_.create(p.rows, p.cols);

#if defined USE_DEPTH
#define PASS1 depths_
#else
#define PASS1 points_
#endif

    volume_->raycast(pose, p.intr, PASS1, normals_);

    if (flag < 1 || flag > 3)
        cuda::renderImage(PASS1, normals_, params_.intr, params_.light_pose, image);
    else if (flag == 2)
        cuda::renderTangentColors(normals_, image);
    else /* if (flag == 3) */
    {
        DeviceArray2D<RGB> i1(p.rows, p.cols, image.ptr(), image.step());
        DeviceArray2D<RGB> i2(p.rows, p.cols, image.ptr() + p.cols, image.step());

        cuda::renderImage(PASS1, normals_, params_.intr, params_.light_pose, i1);
        cuda::renderTangentColors(normals_, i2);
    }
#undef PASS1
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//namespace pcl
//{
//    Eigen::Vector3f rodrigues2(const Eigen::Matrix3f& matrix)
//    {
//        Eigen::JacobiSVD<Eigen::Matrix3f> svd(matrix, Eigen::ComputeFullV | Eigen::ComputeFullU);
//        Eigen::Matrix3f R = svd.matrixU() * svd.matrixV().transpose();

//        double rx = R(2, 1) - R(1, 2);
//        double ry = R(0, 2) - R(2, 0);
//        double rz = R(1, 0) - R(0, 1);

//        double s = sqrt((rx*rx + ry*ry + rz*rz)*0.25);
//        double c = (R.trace() - 1) * 0.5;
//        c = c > 1. ? 1. : c < -1. ? -1. : c;

//        double theta = acos(c);

//        if( s < 1e-5 )
//        {
//            double t;

//            if( c > 0 )
//                rx = ry = rz = 0;
//            else
//            {
//                t = (R(0, 0) + 1)*0.5;
//                rx = sqrt( std::max(t, 0.0) );
//                t = (R(1, 1) + 1)*0.5;
//                ry = sqrt( std::max(t, 0.0) ) * (R(0, 1) < 0 ? -1.0 : 1.0);
//                t = (R(2, 2) + 1)*0.5;
//                rz = sqrt( std::max(t, 0.0) ) * (R(0, 2) < 0 ? -1.0 : 1.0);

//                if( fabs(rx) < fabs(ry) && fabs(rx) < fabs(rz) && (R(1, 2) > 0) != (ry*rz > 0) )
//                    rz = -rz;
//                theta /= sqrt(rx*rx + ry*ry + rz*rz);
//                rx *= theta;
//                ry *= theta;
//                rz *= theta;
//            }
//        }
//        else
//        {
//            double vth = 1/(2*s);
//            vth *= theta;
//            rx *= vth; ry *= vth; rz *= vth;
//        }
//        return Eigen::Vector3d(rx, ry, rz).cast<float>();
//    }
//}

