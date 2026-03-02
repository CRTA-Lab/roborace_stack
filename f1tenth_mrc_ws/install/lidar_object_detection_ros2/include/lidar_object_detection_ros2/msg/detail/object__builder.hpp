// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from lidar_object_detection_ros2:msg/Object.idl
// generated code does not contain a copyright notice

#ifndef LIDAR_OBJECT_DETECTION_ROS2__MSG__DETAIL__OBJECT__BUILDER_HPP_
#define LIDAR_OBJECT_DETECTION_ROS2__MSG__DETAIL__OBJECT__BUILDER_HPP_

#include "lidar_object_detection_ros2/msg/detail/object__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace lidar_object_detection_ros2
{

namespace msg
{

namespace builder
{

class Init_Object_pose
{
public:
  explicit Init_Object_pose(::lidar_object_detection_ros2::msg::Object & msg)
  : msg_(msg)
  {}
  ::lidar_object_detection_ros2::msg::Object pose(::lidar_object_detection_ros2::msg::Object::_pose_type arg)
  {
    msg_.pose = std::move(arg);
    return std::move(msg_);
  }

private:
  ::lidar_object_detection_ros2::msg::Object msg_;
};

class Init_Object_l_shape
{
public:
  explicit Init_Object_l_shape(::lidar_object_detection_ros2::msg::Object & msg)
  : msg_(msg)
  {}
  Init_Object_pose l_shape(::lidar_object_detection_ros2::msg::Object::_l_shape_type arg)
  {
    msg_.l_shape = std::move(arg);
    return Init_Object_pose(msg_);
  }

private:
  ::lidar_object_detection_ros2::msg::Object msg_;
};

class Init_Object_id
{
public:
  Init_Object_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Object_l_shape id(::lidar_object_detection_ros2::msg::Object::_id_type arg)
  {
    msg_.id = std::move(arg);
    return Init_Object_l_shape(msg_);
  }

private:
  ::lidar_object_detection_ros2::msg::Object msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::lidar_object_detection_ros2::msg::Object>()
{
  return lidar_object_detection_ros2::msg::builder::Init_Object_id();
}

}  // namespace lidar_object_detection_ros2

#endif  // LIDAR_OBJECT_DETECTION_ROS2__MSG__DETAIL__OBJECT__BUILDER_HPP_
