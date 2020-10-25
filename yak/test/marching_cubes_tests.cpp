#include <gtest/gtest.h>
#include <yak/mc/marching_cubes.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/ascii_io.h>
#include <pcl/octree/octree_pointcloud_changedetector.h>

static const double OCTREE_RESOLUTION_MM = 0.001;

// Define the baseline mesh for the MarchingCubes.test_simple_meshing unit test.
// The baseline mesh is a dypyramid with 8 triangular faces and 6 vertices.
// This mesh contains no duplicated vertices.
static const double DIPYRAMID_MESH_EXPECTED_VOXEL_SCALE = 0.01;
static const std::string DIPYRAMID_MESH_FILENAME = "dipyramid_ascii.ply";
static const std::string DIPYRAMID_MESH_ASCII = "ply\n\
format ascii 1.0\n\
comment VCGLIB generated\n\
element vertex 6\n\
property float x\n\
property float y\n\
property float z\n\
element face 8\n\
property list uchar int vertex_indices\n\
end_header\n\
0.01 0.005 0.01\n\
0.005 0.01 0.01\n\
0.01 0.01 0.005\n\
0.015 0.01 0.01\n\
0.01 0.01 0.015\n\
0.01 0.015 0.01\n\
3 0 1 2\n\
3 3 0 2\n\
3 0 4 1\n\
3 2 1 5\n\
3 3 4 0\n\
3 5 1 4\n\
3 2 5 3\n\
3 3 5 4\n";

// TODO: A good candidate for the public API.
/**
 * @brief Copy a 16-bit half-float distance value and a 16-bit unsigned int weight value into a 32-bit unsigned int.
 * @param dist 16-bit half-float distance
 * @param weight 16-bit unsigned int weight
 * @param data 32-bit unsigned int voxel value. First 16 bits are the distance, second 16 bits are the weight.
 */
void pack(const half_float::half& dist, const uint16_t& weight, uint32_t& data)
{
  std::memcpy(&data, &dist, 2);
  std::memcpy(((char*)(&data)) + 2, &weight, 2);
}

// TODO: This function is a duplicate of a private member function of yak::TSDFContainer. It would be useful to add it
// to the public API instead.
void unpack(const uint32_t data, half_float::half& dist, uint16_t& weight)
{
  std::memcpy(&dist, &data, 2);
  std::memcpy(&weight, ((char*)(&data)) + 2, 2);
}

///
/// \brief Test that packing and unpacking TSDF voxels is symmetric.
///
TEST(MarchingCubes, pack_unpack)
{
  uint32_t voxel_data;

  half_float::half dist(0.375f);
  uint16_t weight(19);

  pack(dist, weight, voxel_data);

  half_float::half dist_unpack;
  uint16_t weight_unpack;

  unpack(voxel_data, dist_unpack, weight_unpack);

  EXPECT_EQ(float(dist), float(dist_unpack)) << "dist after pack and unpack is different than expected.";
  EXPECT_EQ(weight, weight_unpack) << "weight after pack and unpack is different than expected.";
}

///
/// \brief Test meshing a very simple TSDF voxel volume using marching cubes. Verify that vertices of new mesh match
/// expected output.
/// \todo Add comparison of mesh faces.
///
TEST(MarchingCubes, test_simple_meshing)
{
  auto mc_params = yak::MarchingCubesParameters();
  mc_params.scale = DIPYRAMID_MESH_EXPECTED_VOXEL_SCALE;
  mc_params.min_weight = 1;
  mc_params.clean = true;

  // create a (3 x 3 x 3) voxel volume
  int n_x = 3;
  int n_y = 3;
  int n_z = 3;
  auto volume = yak::TSDFContainer(n_x, n_y, n_z);

  // initialize volume to be all "outside" voxels
  uint32_t empty_voxel;
  pack(half_float::half(1.0), 1, empty_voxel);
  std::vector<uint32_t> voxels(static_cast<std::size_t>(n_x * n_y * n_z), empty_voxel);

  // add one "inside" voxel at the center of the volume
  pack(half_float::half(-1.0), 1, voxels.at(static_cast<std::size_t>(volume.toIndex(1, 1, 1))));

  memcpy(volume.data(), voxels.data(), static_cast<std::size_t>(n_x * n_y * n_z * 4));

  // Generate a new mesh by performing marching cubes on the TSDF volume
  pcl::PolygonMesh mesh_mc = yak::marchingCubesCPU(volume, mc_params);

  // Create a new mesh that we expect to be identical to the one generated by marching cubes
  pcl::PolygonMesh mesh_baseline;
  std::ofstream out(DIPYRAMID_MESH_FILENAME);
  out << DIPYRAMID_MESH_ASCII;
  out.close();
  pcl::io::loadPLYFile(DIPYRAMID_MESH_FILENAME, mesh_baseline);
  remove(DIPYRAMID_MESH_FILENAME.c_str());

  // Expect both the generated mesh and the baseline mesh to have 8 faces
  EXPECT_EQ(mesh_mc.polygons.size(), mesh_baseline.polygons.size()) << "Generated mesh does not have the expected "
                                                                       "number of faces.";

  // Create a point cloud from the vertices of the mesh generated by marching cubes
  pcl::PointCloud<pcl::PointXYZ>::Ptr mesh_mc_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(mesh_mc.cloud, *mesh_mc_cloud);

  // Create a point cloud from the vertices of the predefined baseline mesh
  pcl::PointCloud<pcl::PointXYZ>::Ptr mesh_baseline_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(mesh_baseline.cloud, *mesh_baseline_cloud);

  // Since mc_params.clean = true, expect the mesh from marching cubes to have no duplicate vertices
  EXPECT_EQ(mesh_mc_cloud->size(), mesh_baseline_cloud->size()) << "Generated mesh does not have the expected number "
                                                                   "of vertices.";

  // Use an octree to find vertices in the marching cubes mesh that are not the same as the vertices in the baseline
  pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree(OCTREE_RESOLUTION_MM);
  octree.setInputCloud(mesh_mc_cloud);
  octree.addPointsFromInputCloud();
  octree.switchBuffers();
  octree.setInputCloud(mesh_baseline_cloud);
  octree.addPointsFromInputCloud();

  std::vector<int> different_vertex_indices;
  octree.getPointIndicesFromNewVoxels(different_vertex_indices);

  // Expect zero new indices if the meshes have identical vertices
  EXPECT_EQ(different_vertex_indices.size(), 0) << "Vertices in generated mesh are in different positions than "
                                                   "expected.";
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
