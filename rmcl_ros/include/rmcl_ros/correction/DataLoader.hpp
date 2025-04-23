#ifndef RMCL_ROS_DATA_LOADER_HPP
#define RMCL_ROS_DATA_LOADER_HPP

#include <memory>
#include <rmagine/types/PointCloud.hpp>

#include <rclcpp/rclcpp.hpp>


template<typename MemT> // rmagine Memory Type
class DataLoader_
{
public:
  DataLoader_() {}

  virtual ~DataLoader_() {}

  inline const rmagine::PointCloudView_<MemT> dataset() const
  {
    return rmagine::watch(dataset_);
  }

protected:
  // Data loader fills this (mutex?)
  rmagine::PointCloud_<MemT> dataset_;
  rclcpp::Time stamp_;
};

// default: RAM
using DataLoader = DataLoader_<rmagine::RAM>;

#endif // RMCL_ROS_DATA_LOADER_HPP
