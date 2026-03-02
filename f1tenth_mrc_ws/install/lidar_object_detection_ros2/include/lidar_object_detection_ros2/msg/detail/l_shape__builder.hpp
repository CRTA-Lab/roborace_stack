// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from lidar_object_detection_ros2:msg/LShape.idl
// generated code does not contain a copyright notice

#ifndef LIDAR_OBJECT_DETECTION_ROS2__MSG__DETAIL__L_SHAPE__BUILDER_HPP_
#define LIDAR_OBJECT_DETECTION_ROS2__MSG__DETAIL__L_SHAPE__BUILDER_HPP_

#include "lidar_object_detection_ros2/msg/detail/l_shape__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace lidar_object_detection_ros2
{

namespace msg
{

namespace builder
{

class Init_LShape_l2
{
public:
  explicit Init_LShape_l2(::lidar_object_detection_ros2::msg::LShape & msg)
  : msg_(msg)
  {}
  ::lidar_object_detection_ros2::msg::LShape l2(::lidar_object_detection_ros2::msg::LShape::_l2_type arg)
  {
    msg_.l2 = std::move(arg);
    return std::move(msg_);
  }

private:
  ::lidar_object_detection_ros2::msg::LShape msg_;
};

class Init_LShape_l1
{
public:
  explicit Init_LShape_l1(::lidar_object_detection_ros2::msg::LShape & msg)
  : msg_(msg)
  {}
  Init_LShape_l2 l1(::lidar_object_detection_ros2::msg::LShape::_l1_type arg)
  {
    msg_.l1 = std::move(arg);
    return Init_LShape_l2(msg_);
  }

private:
  ::lidar_object_detection_ros2::msg::LShape msg_;
};

class Init_LShape_theta
{
public:
  explicit Init_LShape_theta(::lidar_object_detection_ros2::msg::LShape & msg)
  : msg_(msg)
  {}
  Init_LShape_l1 theta(::lidar_object_detection_ros2::msg::LShape::_theta_type arg)
  {
    msg_.theta = std::move(arg);
    return Init_LShape_l1(msg_);
  }

private:
  ::lidar_object_detection_ros2::msg::LShape msg_;
};

class Init_LShape_c1
{
public:
  Init_LShape_c1()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_LShape_theta c1(::lidar_object_detection_ros2::msg::LShape::_c1_type arg)
  {
    msg_.c1 = std::move(arg);
    return Init_LShape_theta(msg_);
  }

private:
  ::lidar_object_detection_ros2::msg::LShape msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::lidar_object_detection_ros2::msg::LShape>()
{
  return lidar_object_detection_ros2::msg::builder::Init_LShape_c1();
}

}  // namespace lidar_object_detection_ros2

#endif  // LIDAR_OBJECT_DETECTION_ROS2__MSG__DETAIL__L_SHAPE__BUILDER_HPP_
