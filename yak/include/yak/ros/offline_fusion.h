#ifndef ROS_OFFLINE_FUSION_H
#define ROS_OFFLINE_FUSION_H

#include <Eigen/Dense>
#include <yak/ros/half.hpp>
#include <boost/make_shared.hpp>


// Host
class TSDFContainer
{
public:
  TSDFContainer(int x, int y, int z)
    : data_(boost::make_shared<std::vector<uint32_t>>(x * y * z, 0))
  {
    dims_(0) = x;
    dims_(1) = y;
    dims_(2) = z;
  }

  uint32_t* data() { return data_->data(); }
  const uint32_t* data() const { return data_->data(); }

  void read(int index, half_float::half& dist, uint16_t& weight) const
  {
    unpack((*data_)[index], dist, weight);
  }

  int toIndex(int x, int y, int z) const
  {
    int i = dims_(0) *dims_(1) * z + dims_(0) * y + x;
    return i;
  }

  void unpack(uint32_t data, half_float::half& dist, uint16_t& weight) const
  {
    std::memcpy(&dist, &data, 2);
    std::memcpy(&weight, ((char*)(&data)) + 2, 2);
  }

  int size() const { return dims_(0) * dims_(1) * dims_(2); }

  Eigen::Vector3i dims() const { return dims_; }

  void fromIndex(int index, int& x, int& y, int& z) const
  {
    z = index / (dims_(0) * dims_(1));
    index -= z * dims_(0) * dims_(1);
    y = index / dims_(0);
    x = index % dims_(0);
  }

private:
  boost::shared_ptr<std::vector<uint32_t>> data_;
  Eigen::Vector3i dims_;
};


#endif // ROS_OFFLINE_FUSION_H
