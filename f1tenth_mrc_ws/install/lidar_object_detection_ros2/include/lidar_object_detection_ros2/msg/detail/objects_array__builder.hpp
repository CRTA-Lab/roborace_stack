// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from lidar_object_detection_ros2:msg/ObjectsArray.idl
// generated code does not contain a copyright notice

#ifndef LIDAR_OBJECT_DETECTION_ROS2__MSG__DETAIL__OBJECTS_ARRAY__BUILDER_HPP_
#define LIDAR_OBJECT_DETECTION_ROS2__MSG__DETAIL__OBJECTS_ARRAY__BUILDER_HPP_

#include "lidar_object_detection_ros2/msg/detail/objects_array__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace lidar_object_detection_ros2
{

namespace msg
{

namespace builder
{

class Init_ObjectsArray_objects
{
public:
  explicit Init_ObjectsArray_objects(::lidar_object_detection_ros2::msg::ObjectsArray & msg)
  : msg_(msg)
  {}
  ::lidar_object_detection_ros2::msg::ObjectsArray objects(::lidar_object_detection_ros2::msg::ObjectsArray::_objects_type arg)
  {
    msg_.objects = std::move(arg);
    return std::move(msg_);
  }

private:
  ::lidar_object_detection_ros2::msg::ObjectsArray msg_;
};

class Init_ObjectsArray_header
{
public:
  Init_ObjectsArray_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ObjectsArray_objects header(::lidar_object_detection_ros2::msg::ObjectsArray::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_ObjectsArray_objects(msg_);
  }

private:
  ::lidar_object_detection_ros2::msg::ObjectsArray msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::lidar_object_detection_ros2::msg::ObjectsArray>()
{
  return lidar_object_detection_ros2::msg::builder::Init_ObjectsArray_header();
}

}  // namespace lidar_object_detection_ros2

#endif  // LIDAR_OBJECT_DETECTION_ROS2__MSG__DETAIL__OBJECTS_ARRAY__BUILDER_HPP_
