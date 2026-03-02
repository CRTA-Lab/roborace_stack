// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from lidar_object_detection_ros2:msg/Pose2D.idl
// generated code does not contain a copyright notice

#ifndef LIDAR_OBJECT_DETECTION_ROS2__MSG__DETAIL__POSE2_D__BUILDER_HPP_
#define LIDAR_OBJECT_DETECTION_ROS2__MSG__DETAIL__POSE2_D__BUILDER_HPP_

#include "lidar_object_detection_ros2/msg/detail/pose2_d__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace lidar_object_detection_ros2
{

namespace msg
{

namespace builder
{

class Init_Pose2D_y
{
public:
  explicit Init_Pose2D_y(::lidar_object_detection_ros2::msg::Pose2D & msg)
  : msg_(msg)
  {}
  ::lidar_object_detection_ros2::msg::Pose2D y(::lidar_object_detection_ros2::msg::Pose2D::_y_type arg)
  {
    msg_.y = std::move(arg);
    return std::move(msg_);
  }

private:
  ::lidar_object_detection_ros2::msg::Pose2D msg_;
};

class Init_Pose2D_x
{
public:
  Init_Pose2D_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Pose2D_y x(::lidar_object_detection_ros2::msg::Pose2D::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_Pose2D_y(msg_);
  }

private:
  ::lidar_object_detection_ros2::msg::Pose2D msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::lidar_object_detection_ros2::msg::Pose2D>()
{
  return lidar_object_detection_ros2::msg::builder::Init_Pose2D_x();
}

}  // namespace lidar_object_detection_ros2

#endif  // LIDAR_OBJECT_DETECTION_ROS2__MSG__DETAIL__POSE2_D__BUILDER_HPP_
