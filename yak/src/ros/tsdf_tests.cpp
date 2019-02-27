#include <yak/ros/yak_server.h>
//#include <yak/mc/marching_cubes.h>
#include <yak/offline/test_file.h>

int main()
{
    kfusion::KinFuParams default_params = kfusion::KinFuParams::default_params();
    Eigen::Affine3f world_to_volume (Eigen::Affine3f::Identity());

//    yak::MarchingCubesParameters mc_params;
    yak::FusionServer fusion(default_params, world_to_volume);

    yak::sayHello();

    return 0;
}

//#include <rclcpp/rclcpp.hpp>

//#include <pcl/io/ply_io.h>

//#include <tf2_ros/transform_listener.h>
//#include <tf2_ros/buffer.h>
//#include <tf2_eigen/tf2_eigen.h>

//#include <cv_bridge/cv_bridge.h>

//#include <sensor_msgs/msg/image.hpp>
//#include <geometry_msgs/msg/transform_stamped.hpp>
//#include <std_srvs/srv/trigger.hpp>

//#include <yak/ros/yak_server.h>
//#include <yak/mc/marching_cubes.h>

//static const std::double_t DEFAULT_MINIMUM_TRANSLATION = 0.00001;

///**
// * @brief The OnlineFusionServer class. Integrate depth images into a TSDF volume. When requested, mesh the volume using marching cubes.
// * Note that this will work using both simulated and real robots and depth cameras.
// */
//class FusionNode : public rclcpp::Node
//{
//public:
//  /**
//   * @brief OnlineFusionServer constructor
//   * @param nh - ROS node handle
//   * @param params - KinFu parameters such as TSDF volume size, resolution, etc.
//   * @param world_to_volume - Transform from world frame to volume origin frame.
//   */
//  explicit FusionNode(const kfusion::KinFuParams& params, const Eigen::Affine3f& world_to_volume) :
//    Node("fusion_node"),
//    fusion_(params, world_to_volume),
//    params_(params),
//    clock_(std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME)),
//    tf_buffer_(clock_),
//    robot_tform_listener_(tf_buffer_),
//    world_to_camera_prev_(Eigen::Affine3d::Identity())
//  {
//    // Subscribe to depth images published on the topic named by the depth_topic param. Set up callback to integrate images when received.
//    std::string depth_topic = "image/depth";
////    nh.getParam("depth_topic", depth_topic);

//    auto depth_image_cb = [this](const sensor_msgs::msg::Image::SharedPtr image_in) -> void
//    {
//        // Get the camera pose in the world frame at the time when the depth image was generated.
//        RCLCPP_INFO(this->get_logger(), "Got depth image");
//        geometry_msgs::msg::TransformStamped transform_world_to_camera;
//        try
//        {
//          transform_world_to_camera = tf_buffer_.lookupTransform("base_link", image_in->header.frame_id, tf2::TimePoint(std::chrono::seconds(image_in->header.stamp.sec) + std::chrono::nanoseconds(image_in->header.stamp.nanosec)));
//        }
//        catch(tf2::TransformException &ex)
//        {
//          // Abort integration if tf lookup failed
//         RCLCPP_WARN(this->get_logger(), "%s", ex.what());
//          return;
//        }
//        Eigen::Affine3d world_to_camera = tf2::transformToEigen(transform_world_to_camera);

//        // Find how much the camera moved since the last depth image. If the magnitude of motion was below some threshold, abort integration.
//        // This is to prevent noise from accumulating in the isosurface due to numerous observations from the same pose.
//        std::double_t motion_mag = (world_to_camera.inverse() * world_to_camera_prev_).translation().norm();
//        //ROS_INFO_STREAM(motion_mag);
//        if(motion_mag < DEFAULT_MINIMUM_TRANSLATION)
//        {
//          RCLCPP_INFO(this->get_logger(), "Camera motion below threshold");
//          return;
//        }

//        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image_in, sensor_msgs::image_encodings::TYPE_16UC1);

//        // Integrate the depth image into the TSDF volume
//        if (!fusion_.fuse(cv_ptr->image, world_to_camera.cast<float>()))
//        {
//            RCLCPP_WARN(this->get_logger(), "Failed to fuse image");
//        }

//        // If integration was successful, update the previous camera pose with the new camera pose
//        world_to_camera_prev_ = world_to_camera;
//        return;
//    };

//    depth_image_sub_ = create_subscription<sensor_msgs::msg::Image>(depth_topic, depth_image_cb);

//    auto generate_mesh_cb =
//            [this](const std::shared_ptr<rmw_request_id_t> request_header,
//            const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
//            const std::shared_ptr<std_srvs::srv::Trigger::Response> res) -> void
//    {
//        RCLCPP_INFO(this->get_logger(), "Starting mesh generation");
//        yak::MarchingCubesParameters mc_params;
//        mc_params.scale = params_.volume_resolution;
//        pcl::PolygonMesh mesh = yak::marchingCubesCPU(fusion_.downloadTSDF(), mc_params);
//        RCLCPP_INFO(this->get_logger(), "Meshing done, saving ply");
//        pcl::io::savePLYFileBinary("cubes.ply", mesh);
//        RCLCPP_INFO(this->get_logger(), "Saving done");
//    };

//    // Advertise service for marching cubes meshing
//    generate_mesh_service_ = create_service<std_srvs::srv::Trigger>("generate_mesh_service", generate_mesh_cb);
//  }

//private:
//  rclcpp::Clock::SharedPtr clock_;
//  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_image_sub_;
//  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr generate_mesh_service_;

//  tf2_ros::Buffer tf_buffer_;
//  tf2_ros::TransformListener robot_tform_listener_;

//  yak::FusionServer fusion_;
//  const kfusion::KinFuParams params_;
//  Eigen::Affine3d world_to_camera_prev_;
//};

//int main(int argc, char * argv[])
//{
//    rclcpp::init(argc, argv);


//    kfusion::KinFuParams default_params = kfusion::KinFuParams::default_params();
//    default_params.use_pose_hints = true; // use robot forward kinematics to find camera pose relative to TSDF volume
//    default_params.use_icp = false; // since we're using robot FK to get the camera pose, don't use ICP (TODO: yet!)
//    default_params.update_via_sensor_motion = false;

//    // TODO: convert to ROS 2
//    // Get camera intrinsics from params
//    //  XmlRpc::XmlRpcValue camera_matrix;
//    //  nh.getParam("camera/camera_matrix/data", camera_matrix);
//    //  default_params.intr.fx = static_cast<double>(camera_matrix[0]);
//    //  default_params.intr.fy = static_cast<double>(camera_matrix[4]);
//    //  default_params.intr.cx = static_cast<double>(camera_matrix[2]);
//    //  default_params.intr.cy = static_cast<double>(camera_matrix[5]);

//    // Set up TSDF parameters
//    // TODO: Autocompute resolution from volume length/width/height in meters
//    default_params.volume_dims = cv::Vec3i(640, 640, 640);
//    default_params.volume_resolution = 0.005;
//    default_params.volume_pose = Eigen::Affine3f::Identity(); // This gets overwritten when Yak is initialized
//    default_params.tsdf_trunc_dist = default_params.volume_resolution * 5.0f; //meters;
//    default_params.tsdf_max_weight = 50;   //frames
//    default_params.raycast_step_factor = 0.25;  //in voxel sizes
//    default_params.gradient_delta_factor = 0.25; //in voxel sizes

//    // TODO: Don't hardcode TSDF volume origin pose
//    Eigen::Affine3f world_to_volume (Eigen::Affine3f::Identity());
//    world_to_volume.translation() += Eigen::Vector3f(0.0f, -1.5f, -2.0f);

//    auto node = std::make_shared<FusionNode>(default_params, world_to_volume);

//    rclcpp::spin(node);
//    rclcpp::shutdown();
//    return 0;
//}
