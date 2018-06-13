#ifndef YAK_TSDF_CONTAINER_H
#define YAK_TSDF_CONTAINER_H

#include <Eigen/Dense>
#include <yak/ros/half.hpp>
#include <memory>

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

  void read(int index, half_float::half& dist, uint16_t& weight) const;

  int toIndex(int x, int y, int z) const;

  void fromIndex(int index, int& x, int& y, int& z) const;

  uint32_t* data() { return data_->data(); }
  const uint32_t* data() const { return data_->data(); }

  int size() const;

  Eigen::Vector3i dims() const { return dims_; }

private:
  void unpack(uint32_t data, half_float::half& dist, uint16_t& weight) const;

private:
  std::shared_ptr<std::vector<uint32_t>> data_;
  Eigen::Vector3i dims_;
};

}

#endif // YAK_TSDF_CONTAINER_H
