#ifndef YAK_MARCHING_CUBES_H
#define YAK_MARCHING_CUBES_H

#include <yak/kfusion/tsdf_container.h>
#include <pcl/PolygonMesh.h>

namespace yak
{
struct MarchingCubesParameters
{
  // Scale factor applied to vertices. Usually equal to physical size of a single
  // voxel element.
  double scale = 1.0;

  // Minimum weight of a voxel to not be dropped in the meshing process.
  int min_weight = 4;
};

pcl::PolygonMesh marchingCubesCPU(const yak::TSDFContainer& tsdf, const MarchingCubesParameters& params = {});

}  // namespace yak

#endif
