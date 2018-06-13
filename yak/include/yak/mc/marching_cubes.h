#ifndef MARCHING_CUBES_FUSION_H
#define MARCHING_CUBES_FUSION_H

#include <yak/offline/tsdf_container.h>
#include <pcl/PolygonMesh.h>

namespace yak
{

pcl::PolygonMesh marchingCubesCPU(const yak::TSDFContainer& tsdf);

}

#endif
