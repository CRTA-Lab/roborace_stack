// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from lidar_object_detection_ros2:msg/ScanClusters.idl
// generated code does not contain a copyright notice

#ifndef LIDAR_OBJECT_DETECTION_ROS2__MSG__DETAIL__SCAN_CLUSTERS__BUILDER_HPP_
#define LIDAR_OBJECT_DETECTION_ROS2__MSG__DETAIL__SCAN_CLUSTERS__BUILDER_HPP_

#include "lidar_object_detection_ros2/msg/detail/scan_clusters__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace lidar_object_detection_ros2
{

namespace msg
{

namespace builder
{

class Init_ScanClusters_labels
{
public:
  explicit Init_ScanClusters_labels(::lidar_object_detection_ros2::msg::ScanClusters & msg)
  : msg_(msg)
  {}
  ::lidar_object_detection_ros2::msg::ScanClusters labels(::lidar_object_detection_ros2::msg::ScanClusters::_labels_type arg)
  {
    msg_.labels = std::move(arg);
    return std::move(msg_);
  }

private:
  ::lidar_object_detection_ros2::msg::ScanClusters msg_;
};

class Init_ScanClusters_points
{
public:
  explicit Init_ScanClusters_points(::lidar_object_detection_ros2::msg::ScanClusters & msg)
  : msg_(msg)
  {}
  Init_ScanClusters_labels points(::lidar_object_detection_ros2::msg::ScanClusters::_points_type arg)
  {
    msg_.points = std::move(arg);
    return Init_ScanClusters_labels(msg_);
  }

private:
  ::lidar_object_detection_ros2::msg::ScanClusters msg_;
};

class Init_ScanClusters_header
{
public:
  Init_ScanClusters_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ScanClusters_points header(::lidar_object_detection_ros2::msg::ScanClusters::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_ScanClusters_points(msg_);
  }

private:
  ::lidar_object_detection_ros2::msg::ScanClusters msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::lidar_object_detection_ros2::msg::ScanClusters>()
{
  return lidar_object_detection_ros2::msg::builder::Init_ScanClusters_header();
}

}  // namespace lidar_object_detection_ros2

#endif  // LIDAR_OBJECT_DETECTION_ROS2__MSG__DETAIL__SCAN_CLUSTERS__BUILDER_HPP_
