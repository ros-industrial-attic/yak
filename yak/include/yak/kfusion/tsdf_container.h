#ifndef YAK_TSDF_CONTAINER_H
#define YAK_TSDF_CONTAINER_H

#include <Eigen/Dense>
#include <yak/kfusion/half.hpp>
#include <memory>
#include <vector>

namespace yak
{
/**
 * @brief A container for storing and reading from a dense TSDF grid
 * comprised of 32-bit entries (16 bits of distance in half-format, 16 bits
 * of weight).
 */
class TSDFContainer
{
public:
  TSDFContainer(int x, int y, int z);

  void read(int index, half_float::half& dist, uint16_t& weight) const { unpack((*data_)[index], dist, weight); }

  int toIndex(int x, int y, int z) const
  {
    int i = dims_(0) * dims_(1) * z + dims_(0) * y + x;
    return i;
  }

  void fromIndex(int index, int& x, int& y, int& z) const
  {
    z = index / (dims_(0) * dims_(1));
    index -= z * dims_(0) * dims_(1);
    y = index / dims_(0);
    x = index % dims_(0);
  }

  uint32_t* data() { return data_->data(); }
  const uint32_t* data() const { return data_->data(); }

  int size() const { return dims_(0) * dims_(1) * dims_(2); }

  Eigen::Vector3i dims() const { return dims_; }

private:
  void unpack(uint32_t data, half_float::half& dist, uint16_t& weight) const
  {
    std::memcpy(&dist, &data, 2);
    std::memcpy(&weight, ((char*)(&data)) + 2, 2);
  }

  std::shared_ptr<std::vector<uint32_t>> data_;
  Eigen::Vector3i dims_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace yak

#endif  // YAK_TSDF_CONTAINER_H
