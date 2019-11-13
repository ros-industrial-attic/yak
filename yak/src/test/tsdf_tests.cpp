#include <yak/ros/yak_server.h>
#include <yak/mc/marching_cubes.h>

int main()
{
  kfusion::KinFuParams default_params = kfusion::KinFuParams::default_params();
  Eigen::Affine3f world_to_volume(Eigen::Affine3f::Identity());
  yak::MarchingCubesParameters mc_params;
  yak::FusionServer fusion(default_params, world_to_volume);

  return 0;
}
