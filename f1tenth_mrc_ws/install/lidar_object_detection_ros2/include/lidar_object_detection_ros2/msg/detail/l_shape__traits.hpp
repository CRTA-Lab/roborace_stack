// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from lidar_object_detection_ros2:msg/LShape.idl
// generated code does not contain a copyright notice

#ifndef LIDAR_OBJECT_DETECTION_ROS2__MSG__DETAIL__L_SHAPE__TRAITS_HPP_
#define LIDAR_OBJECT_DETECTION_ROS2__MSG__DETAIL__L_SHAPE__TRAITS_HPP_

#include "lidar_object_detection_ros2/msg/detail/l_shape__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'c1'
#include "lidar_object_detection_ros2/msg/detail/pose2_d__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<lidar_object_detection_ros2::msg::LShape>()
{
  return "lidar_object_detection_ros2::msg::LShape";
}

template<>
inline const char * name<lidar_object_detection_ros2::msg::LShape>()
{
  return "lidar_object_detection_ros2/msg/LShape";
}

template<>
struct has_fixed_size<lidar_object_detection_ros2::msg::LShape>
  : std::integral_constant<bool, has_fixed_size<lidar_object_detection_ros2::msg::Pose2D>::value> {};

template<>
struct has_bounded_size<lidar_object_detection_ros2::msg::LShape>
  : std::integral_constant<bool, has_bounded_size<lidar_object_detection_ros2::msg::Pose2D>::value> {};

template<>
struct is_message<lidar_object_detection_ros2::msg::LShape>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // LIDAR_OBJECT_DETECTION_ROS2__MSG__DETAIL__L_SHAPE__TRAITS_HPP_
