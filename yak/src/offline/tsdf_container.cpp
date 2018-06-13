#include "yak/offline/tsdf_container.h"

yak::TSDFContainer::TSDFContainer(int x, int y, int z)
  : data_(std::make_shared<std::vector<uint32_t>>(x * y * z, 0))
{
  dims_(0) = x;
  dims_(1) = y;
  dims_(2) = z;
}

void yak::TSDFContainer::read(int index, half_float::half& dist, uint16_t& weight) const
{
  unpack((*data_)[index], dist, weight);
}

int yak::TSDFContainer::toIndex(int x, int y, int z) const
{
  int i = dims_(0) * dims_(1) * z + dims_(0) * y + x;
  return i;
}

void yak::TSDFContainer::unpack(uint32_t data, half_float::half& dist, uint16_t& weight) const
{
  std::memcpy(&dist, &data, 2);
  std::memcpy(&weight, ((char*)(&data)) + 2, 2);
}

int yak::TSDFContainer::size() const { return dims_(0) * dims_(1) * dims_(2); }

void yak::TSDFContainer::fromIndex(int index, int& x, int& y, int& z) const
{
  z = index / (dims_(0) * dims_(1));
  index -= z * dims_(0) * dims_(1);
  y = index / dims_(0);
  x = index % dims_(0);
}
