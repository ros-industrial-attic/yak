#ifndef MARCHING_CUBES_FUSION_H
#define MARCHING_CUBES_FUSION_H

#include <yak/ros/offline_fusion.h>
#include <pcl/PolygonMesh.h>

namespace
{

pcl::PolygonMesh marchingCubesCPU(const TSDFContainer& tsdf);

}

#endif
