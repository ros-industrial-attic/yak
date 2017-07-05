#include <iostream>
#include <fstream>

#include <unistd.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include <yak_meshing/GetMesh.h>
#include <yak_meshing/GetMeshRequest.h>
#include <yak_meshing/GetMeshResponse.h>

#include <yak/GetTSDF.h>

#include <yak/GetSparseTSDF.h>

#include <half.hpp>
#include <openvdb/openvdb.h>

#include <openvdb/tools/VolumeToMesh.h>
#include <openvdb/tools/LevelSetUtil.h>
#include <openvdb/tools/MeshToVolume.h>
#include <openvdb/tools/LevelSetSphere.h>
#include <openvdb/util/Util.h>


class GenerateMesh
{
public:
  GenerateMesh(ros::NodeHandle& nh)
  {
    tsdf_client_ = nh.serviceClient<yak::GetTSDF>("/kinfu/get_tsdf");
    tsdf_sparse_client_ = nh.serviceClient<yak::GetSparseTSDF>("/kinfu/get_sparse_tsdf");
    mesh_server_ = nh.advertiseService("get_mesh", &GenerateMesh::GetMesh, this);
   // mesh_server_ = nh.advertiseService("get_mesh", &GenerateMesh::GetMeshFromSparse, this);

    grid = openvdb::FloatGrid::create(/*background value=*/1.0);

  }

//  bool GetMesh(yak_meshing::GetMeshRequest& req, yak_meshing::GetMeshResponse& res) {
//    ROS_INFO("Attempting to get TSDF");
//    yak::GetTSDF srv;
//    if (!tsdf_client_.call(srv))
//    {
//      ROS_ERROR("Couldn't get TSDF");
//      return false;
//    }

//    int sizeX = srv.response.tsdf.size_x;

//    ROS_INFO("Got TSDF");
//    res.value = true;

//    //TODO: Add mesh to service response.

//    int size_x = srv.response.tsdf.size_x;
//    int size_y = srv.response.tsdf.size_y;
//    int size_z = srv.response.tsdf.size_z;

//    int num_voxels_x = srv.response.tsdf.num_voxels_x;
//    int num_voxels_y = srv.response.tsdf.num_voxels_y;
//    int num_voxels_z = srv.response.tsdf.num_voxels_z;

//    int length = num_voxels_x*num_voxels_y*num_voxels_y;

//    std::vector<uint32_t> tsdf_data = srv.response.tsdf.data;

//    ROS_INFO("Got volume info from TSDF");
//    ROS_INFO_STREAM("Max weight: " << srv.response.tsdf.max_weight);

//    //GenerateMesh::MakePointCloud(cloud, tsdf_data, size_x, size_y, size_z, num_voxels_x, num_voxels_y, num_voxels_z);
//    ROS_INFO("Building voxel volume from serialized data...");
//    GenerateMesh::MakeVoxelGrid(grid, tsdf_data, size_x, size_y, size_z, num_voxels_x, num_voxels_y, num_voxels_z);

////    ROS_INFO("Making an example sphere...");
////    GenerateMesh::MakeSphereVoxelGrid(grid);
//    ROS_INFO("Done building volume");

//    ROS_INFO("Meshing voxel volume...");
//    openvdb::tools::VolumeToMesh mesher;
//    mesher.operator()<openvdb::FloatGrid>( grid.operator*() );
//    ROS_INFO("Done meshing volume");
//    WriteMesh("/home/jschornak/clouds/mesh.obj", mesher);
//    //WriteMesh("/home/jschornak/ros/tsdf_ws/src/kinfu_ros/meshes/mesh.obj", mesher);
//    ROS_INFO("Saved .obj to file");

//    return true;
//  }

  //GetMeshFromSparse
  bool GetMesh(yak_meshing::GetMeshRequest& req, yak_meshing::GetMeshResponse& res) {
    ROS_INFO("Attempting to get sparse TSDF");
    yak::GetSparseTSDF srv;
    if (!tsdf_sparse_client_.call(srv))
    {
      ROS_ERROR("Couldn't get sparse TSDF");
      return false;
    }

    int sizeX = srv.response.tsdf.size_x;

    ROS_INFO("Got sparse TSDF");
    res.value = true;

    //TODO: Add mesh to service response.

    float size_x = srv.response.tsdf.size_x;
    float size_y = srv.response.tsdf.size_y;
    float size_z = srv.response.tsdf.size_z;

    int num_voxels_x = srv.response.tsdf.num_voxels_x;
    int num_voxels_y = srv.response.tsdf.num_voxels_y;
    int num_voxels_z = srv.response.tsdf.num_voxels_z;

    int length = num_voxels_x*num_voxels_y*num_voxels_y;

    std::vector<uint32_t> tsdf_data = srv.response.tsdf.data;
    std::vector<uint16_t> row_data = srv.response.tsdf.rows;
    std::vector<uint16_t> col_data = srv.response.tsdf.cols;
    std::vector<uint16_t> sheet_data = srv.response.tsdf.sheets;

    ROS_INFO("Got volume info from TSDF");
    ROS_INFO_STREAM("Max weight: " << srv.response.tsdf.max_weight);

    //GenerateMesh::MakePointCloud(cloud, tsdf_data, size_x, size_y, size_z, num_voxels_x, num_voxels_y, num_voxels_z);
    ROS_INFO("Building voxel volume from serialized sparse data...");
    //GenerateMesh::MakeVoxelGrid(grid, tsdf_data, size_x, size_y, size_z, num_voxels_x, num_voxels_y, num_voxels_z);
    GenerateMesh::MakeVoxelGridFromSparse(grid, tsdf_data, row_data, col_data, sheet_data, size_x, size_y, size_z, num_voxels_x, num_voxels_y, num_voxels_z);

//    ROS_INFO("Making an example sphere...");
//    GenerateMesh::MakeSphereVoxelGrid(grid);
    ROS_INFO("Done building volume");

    ROS_INFO("Meshing voxel volume...");
    openvdb::tools::VolumeToMesh mesher;
    mesher.operator()<openvdb::FloatGrid>( grid.operator*() );
    ROS_INFO("Done meshing volume");
    WriteMesh("/home/jschornak/clouds/mesh.obj", mesher);
    //WriteMesh("/home/jschornak/ros/tsdf_ws/src/kinfu_ros/meshes/mesh.obj", mesher);
    ROS_INFO("Saved .obj to file");

    return true;
  }



  bool GetTSDFData(unsigned int input,  half_float::half& voxelValue, uint16_t& voxelWeight) {
    std::memcpy(&voxelValue, &input, 2);
    std::memcpy(&voxelWeight, ((char*)(&input)) + 2, 2);
    return true;
  }

  bool MakeVoxelGrid(openvdb::FloatGrid::Ptr& grid, std::vector<unsigned int>& data, float size_x, float size_y, float size_z, int res_x, int res_y, int res_z) {
    typedef typename openvdb::FloatGrid::ValueType ValueT;

    openvdb::FloatGrid::Accessor accessor = grid->getAccessor();

    float voxel_dim_x = (float)size_x/(float)res_x;
    //float voxel_dim_y = (float)size_y/(float)res_y;
    //float voxel_dim_z = (float)size_z/(float)res_z;

    ROS_INFO_STREAM("Meters per voxel: " << voxel_dim_x);
    ROS_INFO_STREAM("Voxel count: " << data.size());

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


    bool MakeVoxelGridFromSparse(openvdb::FloatGrid::Ptr& grid, std::vector<unsigned int>& data, std::vector<uint16_t>& rowIndex, std::vector<uint16_t>& colIndex, std::vector<uint16_t>& sheetIndex, float size_x, float size_y, float size_z, int res_x, int res_y, int res_z) {
      typedef typename openvdb::FloatGrid::ValueType ValueT;

      openvdb::FloatGrid::Accessor accessor = grid->getAccessor();

      float voxel_dim_x = (float)size_x/(float)res_x;
      //float voxel_dim_y = (float)size_y/(float)res_y;
      //float voxel_dim_z = (float)size_z/(float)res_z;

      ROS_INFO_STREAM("Meters per voxel in sparse volume: " << voxel_dim_x);

      openvdb::Coord ijk;
      int &i = ijk[0], &j = ijk[1], &k = ijk[2];

      ROS_INFO_STREAM("Nonzero voxel count: " << data.size());
      ROS_INFO_STREAM("Row index count: " << rowIndex.size());
      ROS_INFO_STREAM("Col index count: " << colIndex.size());
      ROS_INFO_STREAM("Sheet index count: " << sheetIndex.size());
      for (int a = 0; a < data.size()-1; a++) {
        i = rowIndex[a];
        j = colIndex[a];
        k = sheetIndex[a];

        uint32_t currentData = data[a];
        //uint32_t currentData = data[res_y*res_z*k + res_y*j + i];
        half_float::half currentValue;
        uint16_t currentWeight;
        //ROS_INFO("Getting TSDF data");
        GetTSDFData(currentData, currentValue, currentWeight);
        ValueT val = ValueT(currentValue);
        //ROS_INFO("Setting OpenVBD voxel");
        //ROS_INFO_STREAM((float)currentValue <<", " << currentWeight << ", " << "["<<i<<","<<j<<","<<k<<"]");
        accessor.setValue(ijk, val);
      }
      ROS_INFO("Scaling voxel volume");
      grid->setTransform(openvdb::math::Transform::createLinearTransform(/*voxel size=*/voxel_dim_x));
    }



    bool MakeSphereVoxelGrid(openvdb::FloatGrid::Ptr& grid) {
      grid = openvdb::tools::createLevelSetSphere<openvdb::FloatGrid>(1.0, openvdb::Vec3f(0,0,0), /*voxel size=*/0.5, /*width=*/4.0);
    }
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
  ros::ServiceClient tsdf_sparse_client_;
  ros::ServiceServer mesh_server_;

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
