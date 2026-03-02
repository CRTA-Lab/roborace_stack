// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from lidar_object_detection_ros2:msg/ScanClusters.idl
// generated code does not contain a copyright notice

#ifndef LIDAR_OBJECT_DETECTION_ROS2__MSG__DETAIL__SCAN_CLUSTERS__TRAITS_HPP_
#define LIDAR_OBJECT_DETECTION_ROS2__MSG__DETAIL__SCAN_CLUSTERS__TRAITS_HPP_

#include "lidar_object_detection_ros2/msg/detail/scan_clusters__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<lidar_object_detection_ros2::msg::ScanClusters>()
{
  return "lidar_object_detection_ros2::msg::ScanClusters";
}

template<>
inline const char * name<lidar_object_detection_ros2::msg::ScanClusters>()
{
  return "lidar_object_detection_ros2/msg/ScanClusters";
}

template<>
struct has_fixed_size<lidar_object_detection_ros2::msg::ScanClusters>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<lidar_object_detection_ros2::msg::ScanClusters>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<lidar_object_detection_ros2::msg::ScanClusters>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // LIDAR_OBJECT_DETECTION_ROS2__MSG__DETAIL__SCAN_CLUSTERS__TRAITS_HPP_
