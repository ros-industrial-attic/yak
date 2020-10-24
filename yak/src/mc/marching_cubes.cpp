#include "yak/mc/marching_cubes.h"
#include "marching_cubes_tables.h"
#include <pcl/io/ply_io.h>
#include <pcl/surface/vtk_smoothing/vtk_utils.h>

#include <vtkSmartPointer.h>
#include <vtkTriangle.h>
#include <vtkCellArray.h>
#include <vtkPolyData.h>
#include <vtkCleanPolyData.h>

#include <omp.h>

namespace
{
struct Triangle
{
  Eigen::Vector3f v[3];
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

float getOffset(float value1, float value2, float value_desired)
{
  double delta = value2 - value1;

  if (delta == 0.0)
  {
    return 0.5;
  }
  return (value_desired - value1) / delta;
}

const static int cubeOffsets[8][3] = {
  { 1, 1, 0 }, { 1, 0, 0 }, { 0, 0, 0 }, { 0, 1, 0 },

  { 1, 1, 1 }, { 1, 0, 1 }, { 0, 0, 1 }, { 0, 1, 1 },
};

std::vector<Triangle> processCube(const yak::TSDFContainer& grid, int x, int y, int z, int min_weight)
{
  // Copy the isovalues of the nearby surfaces into a local array
  float val[8];
  auto read = [&grid](int x, int y, int z, uint16_t& w) {
    half_float::half f;
    grid.read(grid.toIndex(x, y, z), f, w);
    return 5.0f * float(f);
  };

  uint16_t w;
  val[0] = read(x + 1, y + 1, z, w);
  if (w < min_weight)
    return {};
  val[1] = read(x + 1, y, z, w);
  if (w < min_weight)
    return {};
  val[2] = read(x, y, z, w);
  if (w < min_weight)
    return {};
  val[3] = read(x, y + 1, z, w);
  if (w < min_weight)
    return {};

  val[4] = read(x + 1, y + 1, z + 1, w);
  if (w < min_weight)
    return {};
  val[5] = read(x + 1, y, z + 1, w);
  if (w < min_weight)
    return {};
  val[6] = read(x, y, z + 1, w);
  if (w < min_weight)
    return {};
  val[7] = read(x, y + 1, z + 1, w);
  if (w < min_weight)
    return {};

  const static float isolevel = 0.0f;

  // Step 1: We need to compute the 'cubeindex' which indicates which vertices are beneath the iso surface and which
  // are not.
  unsigned int cubeindex = 0u;
  if (val[0] < isolevel)
    cubeindex |= 1;
  if (val[1] < isolevel)
    cubeindex |= 2;
  if (val[2] < isolevel)
    cubeindex |= 4;
  if (val[3] < isolevel)
    cubeindex |= 8;
  if (val[4] < isolevel)
    cubeindex |= 16;
  if (val[5] < isolevel)
    cubeindex |= 32;
  if (val[6] < isolevel)
    cubeindex |= 64;
  if (val[7] < isolevel)
    cubeindex |= 128;

  // Use this cubeindex to look up a table of edges that are cut
  // There are 12 edges and it is cut if the isvalue transitioned from positive to negative (or vica versa)
  // from vertex to vertex
  int edge_flags = yak::edgeFlags[cubeindex];

  // The edge flag info is stored in the first 12 bits of the number. Each bit represents whether or not we need to
  // compute the zero crossing along that edge.
  if (edge_flags == 0)
    return {};  // No edge case (all vertices inside or outside the isosurface)

  // Create storage for all possible edges
  Eigen::Vector3f edges[12];

  // Compute the intersections along each necessary edge
  for (int i = 0; i < 12; ++i)
  {
    if (edge_flags & (1 << i))
    {
      int v0 = yak::edgeConnections[i][0];                   // The first vertex on this edge
      int v1 = yak::edgeConnections[i][1];                   // The second vertex
      float offset = getOffset(val[v0], val[v1], isolevel);  // the ratio of the grid distance along the edge
                                                             // where the intersection takes place
      // Get location of v0
      Eigen::Vector3f v0_pos =
          (Eigen::Vector3i(x, y, z) + Eigen::Vector3i(cubeOffsets[v0][0], cubeOffsets[v0][1], cubeOffsets[v0][2]))
              .cast<float>();
      // Get location of v1
      Eigen::Vector3f v1_pos =
          (Eigen::Vector3i(x, y, z) + Eigen::Vector3i(cubeOffsets[v1][0], cubeOffsets[v1][1], cubeOffsets[v1][2]))
              .cast<float>();
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
    if (triangulation[i * 3] < 0)
      break;  // We're done

    // Otherwise we have 3 more vertices to convert to triangles
    Triangle t;
    for (int j = 0; j < 3; ++j)
    {
      t.v[j] = edges[triangulation[i * 3 + j]];
    }
    triangles.push_back(t);
  }

  return triangles;
}

pcl::PolygonMesh makeMesh(const yak::TSDFContainer& grid, const yak::MarchingCubesParameters& params)
{
  vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
  vtkSmartPointer<vtkCellArray> triangles = vtkSmartPointer<vtkCellArray>::New();
  double s = params.scale;
#pragma omp parallel for
  for (int x = 1; x < grid.dims().x(); ++x)
  {
    for (int y = 1; y < grid.dims().y(); ++y)
    {
      for (int z = 1; z < grid.dims().z(); ++z)
      {
        std::vector<Triangle> ts = processCube(grid, x - 1, y - 1, z - 1, params.min_weight);
#pragma omp critical
        {
          for (auto& t : ts)
          {
            vtkIdType p1 = points->InsertNextPoint(s * static_cast<double>(t.v[0].x()),
                                                   s * static_cast<double>(t.v[0].y()),
                                                   s * static_cast<double>(t.v[0].z()));
            vtkIdType p2 = points->InsertNextPoint(s * static_cast<double>(t.v[1].x()),
                                                   s * static_cast<double>(t.v[1].y()),
                                                   s * static_cast<double>(t.v[1].z()));
            vtkIdType p3 = points->InsertNextPoint(s * static_cast<double>(t.v[2].x()),
                                                   s * static_cast<double>(t.v[2].y()),
                                                   s * static_cast<double>(t.v[2].z()));

            vtkSmartPointer<vtkTriangle> triangle = vtkSmartPointer<vtkTriangle>::New();
            triangle->GetPointIds()->SetId(0, p1);
            triangle->GetPointIds()->SetId(1, p2);
            triangle->GetPointIds()->SetId(2, p3);
            triangles->InsertNextCell(triangle);
          }
        }
      }
    }
  }

  // Create a polydata object
  vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();

  // Add the geometry and topology to the polydata
  polyData->SetPoints(points);
  polyData->SetPolys(triangles);

  pcl::PolygonMesh mesh;
  if (params.clean)
  {
    // Marching cubes produces duplicate vertices so lets clean up the mesh
    vtkSmartPointer<vtkCleanPolyData> cleanPolyData = vtkSmartPointer<vtkCleanPolyData>::New();
    cleanPolyData->SetInputData(polyData);
    cleanPolyData->SetPointMerging(true);
    cleanPolyData->SetConvertPolysToLines(true);
    cleanPolyData->SetConvertLinesToPoints(true);
    cleanPolyData->SetConvertStripsToPolys(true);
    cleanPolyData->Update();

    pcl::VTKUtils::vtk2mesh(cleanPolyData->GetOutput(), mesh);
  }
  else
  {
    pcl::VTKUtils::vtk2mesh(polyData, mesh);
  }

  return mesh;
}

}  // namespace

pcl::PolygonMesh yak::marchingCubesCPU(const yak::TSDFContainer& tsdf, const MarchingCubesParameters& params)
{
  return makeMesh(tsdf, params);
}
