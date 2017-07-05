/*
 * KinfuServer.cpp
 *
 *  Created on: Jun 3, 2015
 *      Author: mklingen
 */

#include <ros/kinfu_server.h>

namespace kfusion
{
    
    KinFuServer::KinFuServer(RosRGBDCamera* camera, const std::string& fixedFrame, const std::string& camFrame) :
            should_exit_(false), camera_(camera), baseFrame_(fixedFrame), cameraFrame_(camFrame)
    {
        raycastImgPublisher_ = camera->nodeHandle.advertise<sensor_msgs::Image>("raycast_image", 10);
        get_tsdf_server_ = camera->nodeHandle.advertiseService("get_tsdf", &KinFuServer::GetTSDF,  this);
        get_sparse_tsdf_server_ = camera->nodeHandle.advertiseService("get_sparse_tsdf", &KinFuServer::GetSparseTSDF,  this);

        tfListener_.waitForTransform("world_frame", "ensenso_sensor_optical_frame", ros::Time::now(), ros::Duration(0.5));
        tfListener_.lookupTransform("world_frame", "ensenso_sensor_optical_frame", ros::Time(0), previous_world_to_sensor_transform_);
        //get_mesh_server_ = camera->nodeHandle.advertiseService("get_mesh", &KinFuServer::GetMesh, this);
    }

    void KinFuServer::PublishRaycastImage()
    {
        const int mode = 3;
        kinfu_->renderImage(viewDevice_, mode);
        
        viewHost_.create(viewDevice_.rows(), viewDevice_.cols(), CV_8UC4);
        viewDevice_.download(viewHost_.ptr<void>(), viewHost_.step);
        std_msgs::Header header;
        header.stamp = ros::Time::now();
        header.frame_id = cameraFrame_;
        cv_bridge::CvImage image(header, std::string("rgba8"), viewHost_);
        raycastImgPublisher_.publish(image.toImageMsg());
    }
    
    void KinFuServer::Update()
    {
        // Try to grab an image from the camera. If it doesn't work, spin once and try again.
        bool has_frame = camera_->Grab(lastDepth_, lastColor_);

        if (!has_frame)
        {
            ros::spinOnce();
            return;
        }

        // Once we have a new image, find the transform between the poses where the current image and the previous image were captured.
        tfListener_.waitForTransform("world_frame", "ensenso_sensor_optical_frame", ros::Time::now(), ros::Duration(0.5));
        tfListener_.lookupTransform("world_frame", "ensenso_sensor_optical_frame", ros::Time(0), current_world_to_sensor_transform_);
//        tf::StampedTransform current_world_to_sensor;
//        tf::StampedTransform past_world_to_sensor;

//        tfListener_.waitForTransform("world_frame", "ensenso_sensor_optical_frame", ros::Time::now(), ros::Duration(0.5));
//        ros::Time present = ros::Time::now();
//        ros::Time past = present - ros::Duration(1.0);
//        tfListener_.lookupTransform("world_frame", "ensenso_sensor_optical_frame", ros::Time(0), current_world_to_sensor);
//        tfListener_.lookupTransform("world_frame", "ensenso_sensor_optical_frame", past, past_world_to_sensor);

        // Seems to log more quickly than sensor returns new positions.

        tf::Transform past_to_current_sensor = previous_world_to_sensor_transform_.inverse() * current_world_to_sensor_transform_;


        //ROS_INFO_STREAM("Sensor transform (X): " << past_to_current_sensor.getOrigin().getX());

        Eigen::Affine3d lastPoseHintTemp;
        tf::transformTFToEigen(past_to_current_sensor, lastPoseHintTemp);
        cv::Mat tempOut(4,4, CV_32F);
        cv::eigen2cv(lastPoseHintTemp.cast<float>().matrix(), tempOut);
        lastPoseHint_ = Affine3f(tempOut);

        ros::Duration timeElapsed = current_world_to_sensor_transform_.stamp_ - previous_world_to_sensor_transform_.stamp_;
        ROS_INFO_STREAM("deltaT: " << timeElapsed << " s");
        double distanceMoved = sqrt(pow(past_to_current_sensor.getOrigin().getX(),2) + pow(past_to_current_sensor.getOrigin().getY(),2) + pow(past_to_current_sensor.getOrigin().getZ(),2));
        ROS_INFO_STREAM("deltaD: " << distanceMoved << " m");
        //ROS_INFO_STREAM("Sensor transform: " << lastPoseHint_.matrix);

        bool has_image = KinFu(lastPoseHint_, lastDepth_, lastColor_);

        if (has_image)
        {
            PublishRaycastImage();
            previous_world_to_sensor_transform_ = current_world_to_sensor_transform_;
        }

        PublishTransform();

    }

    bool KinFuServer::ExecuteBlocking()
    {
        double time_ms = 0;
        bool has_image = false;

        Initialize();

        kfusion::KinFu& kinfu = *kinfu_;

        // TODO: Need to change this to reflect actual sensor refresh rate? (e.g. Ensenso at ~4Hz)
        ROS_INFO("Starting tracking...\n");
        ros::Rate trackHz(30.0f);
        for (int i = 0; !should_exit_ && ros::ok(); ++i)
        {
            Update();
            ros::spinOnce();
            trackHz.sleep();
        }
        return true;
    }

    bool KinFuServer::KinFu(const Affine3f& poseHint, const cv::Mat& depth, const cv::Mat& color)
    {
        depthDevice_.upload(depth.data, depth.step, depth.rows, depth.cols);
        return(* kinfu_)(lastPoseHint_, depthDevice_);
    }

    bool KinFuServer::ConnectCamera()
    {
        bool cameraConnected = false;

        ROS_INFO("Connecting to camera...\n");
        ros::Rate connectionHz(100.0f);
        ros::Timer printTimer;
        printTimer.setPeriod(ros::Duration(1.0));
        printTimer.start();
        while (!cameraConnected)
        {
            cameraConnected = camera_->hasNewDepth;
            ros::spinOnce();
            connectionHz.sleep();
            if (printTimer.hasPending())
            {
                ROS_INFO("No devices connected yet...\n");
            }
        }
        ROS_INFO("Connected.\n");
        return cameraConnected;
    }

    bool KinFuServer::LoadParams()
    {
        KinFuParams params = KinFuParams::default_params();
        params.cols = camera_->GetDepthWidth();
        params.rows = camera_->GetDepthHeight();
        params.intr = camera_->GetDepthIntrinsics();

        LoadParam(params.bilateral_kernel_size, "bilateral_kernel_size");
        LoadParam(params.bilateral_sigma_depth, "bilateral_sigma_depth");
        LoadParam(params.bilateral_sigma_spatial, "bilateral_sigma_spatial");
        LoadParam(params.gradient_delta_factor, "gradient_delta_factor");
        LoadParam(params.icp_angle_thres, "icp_angle_thresh");
        LoadParam(params.icp_dist_thres, "icp_dist_thresh");
        LoadParam(params.icp_iter_num, "icp_iter_num");
        LoadParam(params.icp_truncate_depth_dist, "icp_truncate_depth_dist");
        LoadParam(params.raycast_step_factor, "raycast_step_factor");
        LoadParam(params.tsdf_max_weight, "tsdf_max_weight");
        LoadParam(params.tsdf_min_camera_movement, "tsdf_min_camera_movement");
        LoadParam(params.tsdf_trunc_dist, "tsdf_trunc_dist");
        LoadParam(params.volume_dims.val[0], "volume_dims_x");
        LoadParam(params.volume_dims.val[1], "volume_dims_y");
        LoadParam(params.volume_dims.val[2], "volume_dims_z");
        LoadParam(params.volume_size.val[0], "volume_size_x");
        LoadParam(params.volume_size.val[1], "volume_size_y");
        LoadParam(params.volume_size.val[2], "volume_size_z");

        float volPosX, volPosY, volPosZ;
        volPosX = params.volume_pose.translation().val[0];
        volPosY = params.volume_pose.translation().val[1];
        volPosZ = params.volume_pose.translation().val[2];

        ROS_INFO_STREAM("volPos (default): " << volPosX << ", " << volPosY << ", " << volPosZ);

        LoadParam(volPosX, "volume_pos_x");
        LoadParam(volPosY, "volume_pos_y");
        LoadParam(volPosZ, "volume_pos_z");

        ROS_INFO_STREAM("volPos (loaded): " << volPosX << ", " << volPosY << ", " << volPosZ);
        ROS_INFO_STREAM("translation: " << cv::Affine3f::Vec3(volPosX, volPosY, volPosZ));
// problem's here somewhere????
//        params.volume_pose = Affine3f().translate(Vec3f(volPosX, volPosY,volPosZ));

        params.volume_pose.translate(-params.volume_pose.translation());
        params.volume_pose.translate(cv::Affine3f::Vec3(volPosX, volPosY, volPosZ));


        kinfu_ = KinFu::Ptr(new kfusion::KinFu(params));
        return true;
    }

    bool KinFuServer::PublishTransform()
    {
        tf::StampedTransform currTf;
        currTf.child_frame_id_ = cameraFrame_;
        currTf.frame_id_ = baseFrame_;
        currTf.stamp_ = ros::Time::now();
        cv::Affine3f currPose = kinfu_->getCameraPose();
        currTf.setOrigin(tf::Vector3(currPose.translation().val[0], currPose.translation().val[1], currPose.translation().val[2]));
        cv::Affine3f::Mat3 rot = currPose.rotation();
        tf::Matrix3x3 tfRot(rot.val[0], rot.val[1], rot.val[2], rot.val[3], rot.val[4], rot.val[5], rot.val[6], rot.val[7], rot.val[8]);
        tf::Quaternion tfQuat;
        tfRot.getRotation(tfQuat);
        currTf.setRotation(tfQuat);
        tfBroadcaster_.sendTransform(currTf);
        return true;
    }

    bool KinFuServer::GetTSDF(yak::GetTSDFRequest& req, yak::GetTSDFResponse& res)
    {
        res.tsdf.header.stamp = ros::Time::now();
        res.tsdf.header.frame_id = baseFrame_;
        const kfusion::cuda::TsdfVolume& volume = kinfu_->tsdf();
        res.tsdf.max_weight = volume.getMaxWeight();
        res.tsdf. num_voxels_x = volume.getDims().val[0];
        res.tsdf. num_voxels_y = volume.getDims().val[1];
        res.tsdf. num_voxels_z = volume.getDims().val[2];
        res.tsdf.size_x = volume. getSize().val[0];
        res.tsdf.size_y = volume. getSize().val[1];
        res.tsdf.size_z = volume. getSize().val[2];
        res.tsdf.truncation_dist = volume.getTruncDist();
        res.tsdf.pose.position.x = volume.getPose().translation().val[0];
        res.tsdf.pose.position.y = volume.getPose().translation().val[1];
        res.tsdf.pose.position.z = volume.getPose().translation().val[2];
        cv::Affine3f::Mat3 rot = volume.getPose().rotation();
        tf::Matrix3x3 tfRot(rot.val[0], rot.val[1], rot.val[2], rot.val[3], rot.val[4], rot.val[5], rot.val[6], rot.val[7], rot.val[8]);
        tf::Quaternion tfQuat;
        tfRot.getRotation(tfQuat);
        res.tsdf.pose.orientation.x = tfQuat.x();
        res.tsdf.pose.orientation.y = tfQuat.y();
        res.tsdf.pose.orientation.z = tfQuat.z();
        res.tsdf.pose.orientation.w = tfQuat.w();
        res.tsdf.data.resize(res.tsdf.num_voxels_x * res.tsdf.num_voxels_y * res.tsdf.num_voxels_z, 0);
        ROS_INFO("About to download TSDF data...");
        //uint32_t dataTemp[res.tsdf.num_voxels_x * res.tsdf.num_voxels_y * res.tsdf.num_voxels_z];
        //volume.data().download(dataTemp);

        // res.tsdf.data is a vector<uint32>. Need to figure out how to assign a vector to it en masse.
        //res.tsdf.data.assign(dataTemp);
        volume.data().download(res.tsdf.data.data());
        ROS_INFO("Just downloaded TSDF data");
        // Problem is after this point. Looks like data downloads OK, but when message is sent during callback ROS stops it.
        return true;
    }

    bool KinFuServer::GetSparseTSDF(yak::GetSparseTSDFRequest& req, yak::GetSparseTSDFResponse& res)
    {
        res.tsdf.header.stamp = ros::Time::now();
        res.tsdf.header.frame_id = baseFrame_;
        const kfusion::cuda::TsdfVolume& volume = kinfu_->tsdf();
        res.tsdf.max_weight = volume.getMaxWeight();
        res.tsdf. num_voxels_x = volume.getDims().val[0];
        res.tsdf. num_voxels_y = volume.getDims().val[1];
        res.tsdf. num_voxels_z = volume.getDims().val[2];
        res.tsdf.size_x = volume. getSize().val[0];
        res.tsdf.size_y = volume. getSize().val[1];
        res.tsdf.size_z = volume. getSize().val[2];
        res.tsdf.truncation_dist = volume.getTruncDist();
        res.tsdf.pose.position.x = volume.getPose().translation().val[0];
        res.tsdf.pose.position.y = volume.getPose().translation().val[1];
        res.tsdf.pose.position.z = volume.getPose().translation().val[2];
        cv::Affine3f::Mat3 rot = volume.getPose().rotation();
        tf::Matrix3x3 tfRot(rot.val[0], rot.val[1], rot.val[2], rot.val[3], rot.val[4], rot.val[5], rot.val[6], rot.val[7], rot.val[8]);
        tf::Quaternion tfQuat;
        tfRot.getRotation(tfQuat);
        res.tsdf.pose.orientation.x = tfQuat.x();
        res.tsdf.pose.orientation.y = tfQuat.y();
        res.tsdf.pose.orientation.z = tfQuat.z();
        res.tsdf.pose.orientation.w = tfQuat.w();

        //res.tsdf.data.resize(res.tsdf.num_voxels_x * res.tsdf.num_voxels_y * res.tsdf.num_voxels_z, 0);


        ROS_INFO("Making array to hold data...");
        //uint32_t dataTemp[res.tsdf.num_voxels_x * res.tsdf.num_voxels_y * res.tsdf.num_voxels_z];
        //std::vector<uint32_t> dataTemp[res.tsdf.num_voxels_x * res.tsdf.num_voxels_y * res.tsdf.num_voxels_z];
        std::vector<uint32_t> dataTemp;
        dataTemp.resize(res.tsdf.num_voxels_x * res.tsdf.num_voxels_y * res.tsdf.num_voxels_z);
        ROS_INFO("About to download sparse TSDF data...");
        //uint32_t dataOut[res.tsdf.num_voxels_x * res.tsdf.num_voxels_y * res.tsdf.num_voxels_z];
        volume.data().download(dataTemp.data());
//        std::fill(dataTemp, dataTemp + res.tsdf.num_voxels_x * res.tsdf.num_voxels_y * res.tsdf.num_voxels_z, 0);
//        dataTemp[0]=4209160;
//        dataTemp[1]=4209160;
//        dataTemp[32]=4209160;
//        dataTemp[res.tsdf.num_voxels_x * res.tsdf.num_voxels_y * res.tsdf.num_voxels_z - 32] = 4209160;
        //volume.data().download(res.tsdf.data.data());
        ROS_INFO("Just downloaded TSDF data");


        //Eigen::SparseMatrix<uint32_t> mat


        //4209160

        // Use 3D compressed row notation to convert the dense TSDF serialization to a sparse one.
        //bool dataInCol = false;
        //bool dataInSheet = false;

        std::vector<uint32_t> dataOut;
        std::vector<uint16_t> rows;
        std::vector<uint16_t> cols;
        std::vector<uint16_t> sheets;

        ROS_INFO("Iterating through volume to find nonzero voxels...");
        for (uint16_t i = 0; i < res.tsdf.num_voxels_x; i++) {
          for (uint16_t j = 0; j < res.tsdf.num_voxels_y; j++) {
            for (uint16_t k = 0; k < res.tsdf.num_voxels_z; k++) {
              // Get the contents of every element of the serialized TSDF volume.

              uint32_t currentData = dataTemp[res.tsdf.num_voxels_y*res.tsdf.num_voxels_z*k + res.tsdf.num_voxels_y*j + i];
              half_float::half currentValue;
              uint16_t currentWeight;
              KinFuServer::GetTSDFData(currentData, currentValue, currentWeight);

              // If the weight is nonzero, save the data and row (X) coordinate and flag that the column (Y) and sheet (Z) also contain at least one value.
              if (currentWeight > 0 && currentValue < 1) {
                dataOut.push_back(currentData);
                rows.push_back(i);
                cols.push_back(j);
                sheets.push_back(k);

//                if (dataInCol == false){
//                  // first data in column
//                  // Add index of the row of the first element in this column
//                  cols.push_back(rows.size()-1);
//                }
//                if (dataInCol == false) {
//                  // first data in sheet
//                  // Add index of the row of the first element in this sheet
//                  sheets.push_back(rows.size()-1);
//                }

                //dataInCol = true;
                //dataInSheet = true;
              }
            }
            //dataInCol = false;
          }
          //dataInSheet = false;
        }

        // Resize the arrays in the message to match the actual lengths of the vectors.
        // Assign data to the SparseTSDF message.
        ROS_INFO("Assigning data to result...");
        res.tsdf.data.resize(dataOut.size());
        std::vector<uint32_t>::iterator itA;
        itA = dataOut.begin();
        res.tsdf.data.assign(itA, dataOut.end());

        res.tsdf.rows.resize(rows.size());
        std::vector<uint16_t>::iterator itB;
        itB = rows.begin();
        res.tsdf.rows.assign(itB, rows.end());

        res.tsdf.cols.resize(cols.size());
        itB = cols.begin();
        res.tsdf.cols.assign(itB, cols.end());

        res.tsdf.sheets.resize(sheets.size());
        itB = sheets.begin();
        res.tsdf.sheets.assign(itB, sheets.end());
        ROS_INFO("Created sparse TSDF structure");

        //KinFuServer::TruncateTSDF(dataTemp, res.tsdf.num_voxels_x, res.tsdf.num_voxels_y, res.tsdf.num_voxels_z);

        // res.tsdf.data is a vector<uint32>. Need to figure out how to assign a vector to it en masse.
        //res.tsdf.data.assign(dataTemp);
        //volume.data().download(res.tsdf.data.data());
        return true;
    }

    bool KinFuServer::GetTSDFData(uint32_t input,  half_float::half& voxelValue, uint16_t& voxelWeight) {
      std::memcpy(&voxelValue, &input, 2);
      std::memcpy(&voxelWeight, ((char*)(&input)) + 2, 2);
      return true;
    }

//    bool KinFuServer::TruncateTSDF(std::vector<uint32_t> &data, std::vector<uint32_t> &dataOut, std::vector<uint16_t> &rows, std::vector<uint16_t> &cols, std::vector<uint16_t> &sheets, int numVoxelsX, int numVoxelsY, int numVoxelsZ)
//    {
//      std::vector<uint32_t> dataOut;
//      std::vector<uint16_t> rows;
//      std::vector<uint16_t> cols;
//      std::vector<uint16_t> sheets;

//      bool dataInCol = false;
//      bool dataInSheet = false;

//      for (uint16_t k = 0; k < numVoxelsZ; k++) {
//        for (uint16_t j = 0; j < numVoxelsY; j++) {
//          for (uint16_t i = 0; i < numVoxelsX; i++) {
//            uint32_t currentData = data[numVoxelsY*numVoxelsZ*k + numVoxelsY*j + i];
//            half_float::half currentValue;
//            //float currentValue;

//            uint16_t currentWeight;

//            GetTSDFData(currentData, currentValue, currentWeight);
//  //          ROS_INFO_STREAM("Current weight: " << currentWeight);
//            if (currentWeight > 0) {
//              dataOut.push_back(currentData);
//              rows.push_back(i);
//              dataInCol = true;
//              dataInSheet = true;
//            }
//          }
//          if (dataInCol) {
//            cols.push_back(j);
//            dataInCol = false;
//          }
//        }
//        if (dataInSheet) {
//          sheets.push_back(k);
//          dataInSheet = false;
//        }
//      }

//      return true;
//    }



//    bool MakeTSDFData(uint32_t& output,  half_float::half voxelValue, uint16_t voxelWeight) {
//      std::memcpy(((char*)(&output)), voxelValue, 2);
//      std::memcpy(((char*)(&output)) + 2, voxelWeight, 2);
//      return true;
//    }

    /*
    bool KinFuServer::GetMesh(yak::GetMeshRequest& req, yak::GetMeshResponse& res)
    {
      ROS_INFO("Hit service callback");
      //const kfusion::cuda::TsdfVolume& volume = kinfu_->tsdf();

      // pcl::cuda is deprecated. Not sure if casting to pcl::gpu will work but it's worth a shot!
      //const pcl::gpu::TsdfVolume& volume = (pcl::gpu::TsdfVolume&)(kinfu_->tsdf());
      const kfusion::cuda::TsdfVolume& volume = kinfu_->tsdf();
      ROS_INFO("Loaded TSDF volume");

      int dim_x = volume.getDims().val[0];
      int dim_y = volume.getDims().val[1];
      int dim_z = volume.getDims().val[2];
      ROS_INFO("Got existing volume dimensions");

      const Eigen::Vector3i resolution(dim_x, dim_y, dim_z);
      ROS_INFO("Created resolution vector");

      const pcl::gpu::TsdfVolume volume_temp(resolution);
      const pcl::gpu::TsdfVolume& pcl_volume = volume_temp;
      ROS_INFO("Made a pcl tsdf volume");

      // Want to intiialize triangles_buffer to be empty so MarchingCubes uses its default buffer.
      pcl::gpu::DeviceArray<pcl::PointXYZ> triangles_buffer;
      ROS_INFO("Initialized triangles buffer");
      pcl::gpu::DeviceArray<pcl::PointXYZ>& buffer = triangles_buffer;
      ROS_INFO("Setup buffer reference");

      // Run marching cubes on the provided TSDF volume, with results output to the buffer.
      // Throws some errors right now.
      //run(const TsdfVolume& tsdf, DeviceArray<PointType>& triangles_buffer);

      pcl::gpu::MarchingCubes cubes;
      ROS_INFO("Initialized marching cubes");

      //cubes.run(pcl_volume, buffer);
      //ROS_INFO("Ran marching cubes on pcl tsdf volume");

      cubes.run((pcl::gpu::TsdfVolume&)volume, buffer);
      ROS_INFO("Ran marching cubes on kinfu tsdf volume");

      res.value = true;
      ROS_INFO("Done with service!");
      return true;
    }
    */


} /* namespace kfusion */


