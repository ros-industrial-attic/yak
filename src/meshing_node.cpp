#include <iostream>
#include <fstream>

#include <bitset>

#include <unistd.h>

//#include <opencv2/imgproc/imgproc.hpp>
//#include <ros/ros_rgbd_camera.hpp>

//#include <sensor_msgs/Image.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include <yak/GetMesh.h>
#include <yak/GetMeshRequest.h>
#include <yak/GetMeshResponse.h>

#include <yak/GetTSDF.h>

#include <ros/half.hpp>
#include <openvdb/openvdb.h>

//#include <pcl_ros/point_cloud.h>
//#include <pcl/point_types.h>
//#include <pcl_conversions/pcl_conversions.h>
//#include <pcl/point_cloud.h>
//#include <pcl/surface/marching_cubes_hoppe.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/console/parse.h>
//#include <pcl/common/transforms.h>
//#include <pcl/common/projection_matrix.h>



#include <openvdb/tools/VolumeToMesh.h>
#include <openvdb/tools/LevelSetUtil.h>
#include <openvdb/tools/MeshToVolume.h>
#include <openvdb/tools/LevelSetSphere.h>
#include <openvdb/util/Util.h>

//#include <pcl_visualization/cloud_viewer.h>

//using namespace kfusion;

//using half_float::half;

//typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class GenerateMesh
{
public:
  GenerateMesh(ros::NodeHandle& nh)
  {
    tsdf_client_ = nh.serviceClient<yak::GetTSDF>("/kinfu/get_tsdf");
    mesh_server_ = nh.advertiseService("get_mesh", &GenerateMesh::GetMesh, this);
    point_cloud_publisher_ = nh.advertise<sensor_msgs::PointCloud2>("tsdf_cloud", 1);

    vis_pub_ = nh.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );

    grid = openvdb::FloatGrid::create(/*background value=*/1.0);

  }

  bool GetMesh(yak::GetMeshRequest& req, yak::GetMeshResponse& res) {
    ROS_INFO("Attempting to get TSDF");
    yak::GetTSDF srv;
    if (!tsdf_client_.call(srv))
    {
      ROS_ERROR("Couldn't get TSDF");
      return false;
    }

    int sizeX = srv.response.tsdf.size_x;

    ROS_INFO("Got TSDF");
    res.value = true;

    //TODO: Add mesh to service response.

    int size_x = srv.response.tsdf.size_x;
    int size_y = srv.response.tsdf.size_y;
    int size_z = srv.response.tsdf.size_z;

    int num_voxels_x = srv.response.tsdf.num_voxels_x;
    int num_voxels_y = srv.response.tsdf.num_voxels_y;
    int num_voxels_z = srv.response.tsdf.num_voxels_z;

    int length = num_voxels_x*num_voxels_y*num_voxels_y;

    std::vector<unsigned int> tsdf_data = srv.response.tsdf.data;

    ROS_INFO("Got volume info from TSDF");
    ROS_INFO_STREAM("Max weight: " << srv.response.tsdf.max_weight);

    //GenerateMesh::MakePointCloud(cloud, tsdf_data, size_x, size_y, size_z, num_voxels_x, num_voxels_y, num_voxels_z);
    ROS_INFO("Building voxel volume from serialized data...");
    GenerateMesh::MakeVoxelGrid(grid, tsdf_data, size_x, size_y, size_z, num_voxels_x, num_voxels_y, num_voxels_z);
//    ROS_INFO("Making an example sphere...");
//    GenerateMesh::MakeSphereVoxelGrid(grid);
    ROS_INFO("Done building volume");
    //ROS_INFO("Finished reading point cloud from TSDF");
    // Max cloud size at 512^3 is 134,217,728

    //pcl::io::savePCDFileASCII ("./clouds/test_pcd.pcd", cloud);
    //pcl::io::savePCDFileASCII ("/home/jschornak/clouds/test_pcd.pcd", cloud);


   // ROS_INFO("Saved point cloud");

    ROS_INFO("Meshing voxel volume...");
    openvdb::tools::VolumeToMesh mesher;
    mesher.operator()<openvdb::FloatGrid>( grid.operator*() );
    ROS_INFO("Done meshing volume");
    WriteMesh("/home/jschornak/clouds/mesh.obj", mesher);
    //WriteMesh("/home/jschornak/ros/tsdf_ws/src/yak/meshes/mesh.obj", mesher);
    ROS_INFO("Saved .obj to file");

//    visualization_msgs::Marker marker;
//    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
//    //marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
//    marker.mesh_resource = "package://yak/meshes/mesh.stl";

//    marker.header.frame_id = "camera_depth_optical_frame";
//    marker.header.stamp = ros::Time();
//    marker.ns = "meshing_node";
//    marker.id = 0;
//   // marker.type = visualization_msgs::Marker::SPHERE;
//    marker.action = visualization_msgs::Marker::ADD;
//    marker.pose.position.x = 1;
//    marker.pose.position.y = 1;
//    marker.pose.position.z = 1;
//    marker.pose.orientation.x = 0.0;
//    marker.pose.orientation.y = 0.0;
//    marker.pose.orientation.z = 0.0;
//    marker.pose.orientation.w = 1.0;
//    marker.scale.x = 1;
//    marker.scale.y = 0.1;
//    marker.scale.z = 0.1;
//    marker.color.a = 1.0; // Don't forget to set the alpha!
//    marker.color.r = 0.0;
//    marker.color.g = 1.0;
//    marker.color.b = 0.0;

//    vis_pub_.publish(marker);

    //ROS_INFO("Updated point cloud publisher");
    //sensor_msgs::PointCloud2 cloud_msg;
    //pcl::toROSMsg(cloud, cloud_msg);
    //std::string frame_id = "camera_depth_optical_frame";
    //cloud_msg.header.frame_id = cloud_msg.header.frame_id.empty() ? frame_id : cloud_msg.header.frame_id;
    //point_cloud_publisher_.publish(cloud_msg);

    return true;
  }



  bool GetTSDFData(unsigned int input,  half_float::half& voxelValue, uint16_t& voxelWeight) {
    std::memcpy(&voxelValue, &input, 2);
    std::memcpy(&voxelWeight, ((char*)(&input)) + 2, 2);
    return true;
  }

//  bool MakePointCloud(pcl::PointCloud<pcl::PointXYZ>& cloud, std::vector<unsigned int>& data, int size_x, int size_y, int size_z, int res_x, int res_y, int res_z) {
//    float voxel_dim_x = (float)size_x/(float)res_x;
//    float voxel_dim_y = (float)size_y/(float)res_y;
//    float voxel_dim_z = (float)size_z/(float)res_z;

//    ROS_INFO_STREAM("Voxel X dim: " << voxel_dim_x);

//    for (int i = 0; i < res_x; i++) {
//      for (int j = 0; j < res_y; j++) {
//        for (int k = 0; k < res_z; k++) {
//          int currentData = data[res_y*res_z*k + res_y*j + i];
//          half_float::half currentValue;
//          int currentWeight;

//          GetTSDFData(currentData, currentValue, currentWeight);
//          if (currentWeight > 40 && currentValue < 0.1 && currentValue > -0.1) {
//            //ROS_INFO_STREAM("Found an occupied voxel at (" << i << ", " << j << ", " << k << ")");

//            pcl::PointXYZ currentPoint((float)i*voxel_dim_x, (float)j*voxel_dim_y, (float)k*voxel_dim_z);
//            cloud.push_back(currentPoint);
//            //usleep(100000);
//          }
//        }
//      }
//    }
//    ROS_INFO_STREAM("Made a cloud of size " << cloud.points.size());
//    ROS_INFO_STREAM("The first point is at " << cloud.points[0].x << " " << cloud.points[0].y << " " << cloud.points[0].z);

//    return true;
//  }

  bool MakeVoxelGrid(openvdb::FloatGrid::Ptr& grid, std::vector<unsigned int>& data, int size_x, int size_y, int size_z, int res_x, int res_y, int res_z) {
    typedef typename openvdb::FloatGrid::ValueType ValueT;

    openvdb::FloatGrid::Accessor accessor = grid->getAccessor();
    //openvdb::FloatGrid::Ptr grid = openvdb::FloatGrid::create(/*background value=*/2.0);

    float voxel_dim_x = (float)size_x/(float)res_x;
    //float voxel_dim_y = (float)size_y/(float)res_y;
    //float voxel_dim_z = (float)size_z/(float)res_z;

    ROS_INFO_STREAM("Meters per voxel: " << voxel_dim_x);

    openvdb::Coord ijk;
    int &i = ijk[0], &j = ijk[1], &k = ijk[2];

    for (i = 0; i < res_x; i++) {
      for (j = 0; j < res_y; j++) {
        for (k = 0; k < res_z; k++) {
          int currentData = data[res_y*res_z*k + res_y*j + i];
          half_float::half currentValue;
          //float currentValue;

          uint16_t currentWeight;

          GetTSDFData(currentData, currentValue, currentWeight);
          ValueT val = ValueT(currentValue);
//          ROS_INFO_STREAM("Current weight: " << currentWeight);
          if (currentWeight > 0) {
            accessor.setValue(ijk, val);
          }
          }
        }
      }
    grid->setTransform(openvdb::math::Transform::createLinearTransform(/*voxel size=*/voxel_dim_x));
    }




    bool MakeSphereVoxelGrid(openvdb::FloatGrid::Ptr& grid) {
      grid = openvdb::tools::createLevelSetSphere<openvdb::FloatGrid>(1.0, openvdb::Vec3f(0,0,0), /*voxel size=*/0.5, /*width=*/4.0);
    }

    //pcl::PointCloud<pcl::PointXYZ> cloud;
    openvdb::FloatGrid::Ptr grid;



private:
  void WriteMesh(const char* filename,
      openvdb::tools::VolumeToMesh &mesh ){

    std::ofstream file;
    file.open(filename);

    openvdb::tools::PointList *verts = &mesh.pointList();
    openvdb::tools::PolygonPoolList *polys = &mesh.polygonPoolList();

    for( size_t i = 0; i < mesh.pointListSize(); i++ ){
      openvdb::Vec3s &v = (*verts)[i];
      file << "v " << v[0] << " " << v[1] << " " << v[2] << std::endl;
    }

    for( size_t i = 0; i < mesh.polygonPoolListSize(); i++ ){

      for( size_t ndx = 0; ndx < (*polys)[i].numTriangles(); ndx++ ){
        openvdb::Vec3I *p = &((*polys)[i].triangle(ndx));
        file << "f " << p->x()+1 << " " << p->y()+1 << " " << p->z()+1 << std::endl;
      }

      for( size_t ndx = 0; ndx < (*polys)[i].numQuads(); ndx++ ){
        openvdb::Vec4I *p = &((*polys)[i].quad(ndx));
        file << "f " << p->x()+1 << " " << p->y()+1 << " " << p->z()+1 << " " << p->w()+1 << std::endl;
      }
    }

    file.close();
  }


  ros::ServiceClient tsdf_client_;
  ros::ServiceServer mesh_server_;
  ros::Publisher point_cloud_publisher_;
  ros::Publisher vis_pub_;

};


int main(int argc, char* argv[])
{
    openvdb::initialize();
    ros::init(argc, argv, "meshing_node");
    ros::NodeHandle nh;

    GenerateMesh app(nh);

    ros::spin();

    return 0;
}
