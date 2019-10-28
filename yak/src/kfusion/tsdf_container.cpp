#include <yak/kfusion/tsdf_container.h>

yak::TSDFContainer::TSDFContainer(int x, int y, int z) : data_(std::make_shared<std::vector<uint32_t>>(x * y * z, 0))
{
  dims_(0) = x;
  dims_(1) = y;
  dims_(2) = z;
}
