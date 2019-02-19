#include "marching_cubes_tables.h"

#include <pcl/geometry/polygon_mesh.h>
#include <pcl/io/ply_io.h>

#include <memory>

template<int X, int Y, int Z>
struct ScalarField
{
  const static int SIZE_X = X;
  const static int SIZE_Y = Y;
  const static int SIZE_Z = Z;
  float data[X][Y][Z];
};

using Grid = ScalarField<256, 256, 256>;


std::unique_ptr<Grid> makeScene()
{
  std::unique_ptr<Grid> grid (new Grid);

  Eigen::Vector3d sphere_center (grid->SIZE_X / 2, grid->SIZE_Y / 2, grid->SIZE_Z / 2);
  double radius = grid->SIZE_X / 10;
  // Sample a sphere at each grid position
  // The origin of each cell is at the bottom corner of each cell

  for (int x = 0; x < Grid::SIZE_X; ++x)
  {
    for (int y = 0; y < Grid::SIZE_Y; ++y)
    {
      for (int z = 0; z < Grid::SIZE_Z; ++z)
      {
        Eigen::Vector3d sample_point (x, y, z);

        double signed_dist = (sample_point - sphere_center).norm() - radius;
        grid->data[x][y][z] = signed_dist;
      }
    }
  }

  return grid;
}

std::unique_ptr<Grid> makeScene2()
{
  std::unique_ptr<Grid> grid (new Grid);

  Eigen::Vector3d sphere1_center (250, 500, 500);
  Eigen::Vector3d sphere2_center (400, 500, 500);

  double radius = 100.0;
  // Sample a sphere at each grid position
  // The origin of each cell is at the bottom corner of each cell

  for (int x = 0; x < Grid::SIZE_X; ++x)
  {
    for (int y = 0; y < Grid::SIZE_Y; ++y)
    {
      for (int z = 0; z < Grid::SIZE_Z; ++z)
      {
        Eigen::Vector3d sample_point (x, y, z);

        double signed_dist1 = (sample_point - sphere1_center).norm() - radius;
        double signed_dist2 = (sample_point - sphere2_center).norm() - radius;
        grid->data[x][y][z] = std::min(signed_dist1, signed_dist2);
      }
    }
  }

  return grid;
}

struct Triangle
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3f v[3];
};

float getOffset(float value1, float value2, float value_desired)
{
  double delta = value2 - value1;

  if(delta == 0.0)
  {
    return 0.5;
  }
  return (value_desired - value1)/delta;
}

const static int cubeOffsets[8][3] = {
  {1, 1, 0},
  {1, 0, 0},
  {0, 0, 0},
  {0, 1, 0},

  {1, 1, 1},
  {1, 0, 1},
  {0, 0, 1},
  {0, 1, 1},
};

std::vector<Triangle> processCube(const Grid& grid, int x, int y, int z)
{

  // Copy the isovalues of the nearby surfaces into a local array
  float val[8];
  // bottom face
  val[0] = grid.data[x+1][y+1][z];
  val[1] = grid.data[x+1][y][z];
  val[2] = grid.data[x][y][z];
  val[3] = grid.data[x][y+1][z];
  // top face
  val[4] = grid.data[x+1][y+1][z+1];
  val[5] = grid.data[x+1][y][z+1];
  val[6] = grid.data[x][y][z+1];
  val[7] = grid.data[x][y+1][z+1];

  const static float isolevel = 0.0f;

  // Step 1: We need to compute the 'cubeindex' which indicates which vertices are beneath the iso surface and which
  // are not.
  unsigned int cubeindex = 0u;
  if (val[0] < isolevel) cubeindex |= 1;
  if (val[1] < isolevel) cubeindex |= 2;
  if (val[2] < isolevel) cubeindex |= 4;
  if (val[3] < isolevel) cubeindex |= 8;
  if (val[4] < isolevel) cubeindex |= 16;
  if (val[5] < isolevel) cubeindex |= 32;
  if (val[6] < isolevel) cubeindex |= 64;
  if (val[7] < isolevel) cubeindex |= 128;

  // Use this cubeindex to look up a table of edges that are cut
  // There are 12 edges and it is cut if the isvalue transitioned from positive to negative (or vica versa)
  // from vertex to vertex
  int edge_flags = yak::edgeFlags[cubeindex];

  // The edge flag info is stored in the first 12 bits of the number. Each bit represents whether or not we need to
  // compute the zero crossing along that edge.
  if (edge_flags == 0) return {}; // No edge case (all vertices inside or outside the isosurface)

  // Create storage for all possible edges
  Eigen::Vector3f edges[12];

  // Compute the intersections along each necessary edge
  for (int i = 0; i < 12; ++i)
  {
    if (edge_flags & (1 << i))
    {
      int v0 = yak::edgeConnections[i][0]; // The first vertex on this edge
      int v1 = yak::edgeConnections[i][1]; // The second vertex
      float offset = getOffset(val[v0], val[v1], isolevel); // the ratio of the grid distance along the edge
                                                            // where the intersection takes place
      // Get location of v0
      Eigen::Vector3f v0_pos = (Eigen::Vector3i(x,y,z) + Eigen::Vector3i(cubeOffsets[v0][0], cubeOffsets[v0][1], cubeOffsets[v0][2])).cast<float>();
      // Get location of v1
      Eigen::Vector3f v1_pos = (Eigen::Vector3i(x,y,z) + Eigen::Vector3i(cubeOffsets[v1][0], cubeOffsets[v1][1], cubeOffsets[v1][2])).cast<float>();
      // Interpolate and update that edge index
      edges[i] = v0_pos + offset * (v1_pos - v0_pos);
    }
  }

  // Now look up the triangulation for this cubeindex. This returns a number with up to 15 bits. Each set of
  // three subsequent bits represents the edge intersection that will make up this triangle. The list terminates
  // with a negative one (-1).
  const int* triangulation = yak::triangleTable[cubeindex];

  // Up to 5 triangles
  std::vector<Triangle> triangles;

  for (int i = 0; i < 5; ++i)
  {
    if (triangulation[i * 3] < 0) break; // We're done

    // Otherwise we have 3 more vertices to convert to triangles
    Triangle t;
    for (int j = 0; j < 3; ++j)
    {
      t.v[j] = edges[triangulation[i*3 + j]];
    }
    triangles.push_back(t);
  }

  return triangles;
}

pcl::geometry::PolygonMesh makeMesh(const Grid& grid)
{
  // For each cube inside the grid, let's generate triangles...
  std::vector<Triangle> triangles;

  for (int x = 1; x < Grid::SIZE_X; ++x)
  {
    for (int y = 1; y < Grid::SIZE_Y; ++y)
    {
      for (int z = 1; z < Grid::SIZE_Z; ++z)
      {
        std::vector<Triangle> ts = processCube(grid, x - 1, y - 1, z - 1);
        triangles.insert(triangles.end(), ts.begin(), ts.end());
      }
    }
  }

  // Now we have polygon soup. Let's add em all to the polygon mesh
  pcl::geometry::PolygonMesh mesh;
  mesh.polygons.resize(triangles.size()); // We have N_TRIANGLES number of polygons, each of size 3
  pcl::PointCloud<pcl::PointXYZ> vertices;
  vertices.resize(triangles.size() * 3); // We have 3 times triangles number of vertices

  for (unsigned i = 0; i < triangles.size(); ++i)
  {
    auto idx = i * 3;
    auto& verts = mesh.polygons[i];
    verts.vertices = {idx, idx+1, idx+2};

    vertices[idx+0] = pcl::PointXYZ(triangles[i].v[0].x(), triangles[i].v[0].y(), triangles[i].v[0].z());
    vertices[idx+1] = pcl::PointXYZ(triangles[i].v[1].x(), triangles[i].v[1].y(), triangles[i].v[1].z());
    vertices[idx+2] = pcl::PointXYZ(triangles[i].v[2].x(), triangles[i].v[2].y(), triangles[i].v[2].z());
  }

  pcl::toPCLPointCloud2(vertices, mesh.cloud);

  return mesh;
}

int main()
{
  std::unique_ptr<Grid> g = makeScene();
  pcl::PolygonMesh mesh = makeMesh(*g);

  pcl::io::savePLYFileBinary("mesh.ply", mesh);
}
