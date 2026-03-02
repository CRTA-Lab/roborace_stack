// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from lidar_object_detection_ros2:msg/LShape.idl
// generated code does not contain a copyright notice

#ifndef LIDAR_OBJECT_DETECTION_ROS2__MSG__DETAIL__L_SHAPE__STRUCT_HPP_
#define LIDAR_OBJECT_DETECTION_ROS2__MSG__DETAIL__L_SHAPE__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'c1'
#include "lidar_object_detection_ros2/msg/detail/pose2_d__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__lidar_object_detection_ros2__msg__LShape __attribute__((deprecated))
#else
# define DEPRECATED__lidar_object_detection_ros2__msg__LShape __declspec(deprecated)
#endif

namespace lidar_object_detection_ros2
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct LShape_
{
  using Type = LShape_<ContainerAllocator>;

  explicit LShape_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : c1(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->theta = 0.0f;
      this->l1 = 0.0f;
      this->l2 = 0.0f;
    }
  }

  explicit LShape_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : c1(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->theta = 0.0f;
      this->l1 = 0.0f;
      this->l2 = 0.0f;
    }
  }

  // field types and members
  using _c1_type =
    lidar_object_detection_ros2::msg::Pose2D_<ContainerAllocator>;
  _c1_type c1;
  using _theta_type =
    float;
  _theta_type theta;
  using _l1_type =
    float;
  _l1_type l1;
  using _l2_type =
    float;
  _l2_type l2;

  // setters for named parameter idiom
  Type & set__c1(
    const lidar_object_detection_ros2::msg::Pose2D_<ContainerAllocator> & _arg)
  {
    this->c1 = _arg;
    return *this;
  }
  Type & set__theta(
    const float & _arg)
  {
    this->theta = _arg;
    return *this;
  }
  Type & set__l1(
    const float & _arg)
  {
    this->l1 = _arg;
    return *this;
  }
  Type & set__l2(
    const float & _arg)
  {
    this->l2 = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    lidar_object_detection_ros2::msg::LShape_<ContainerAllocator> *;
  using ConstRawPtr =
    const lidar_object_detection_ros2::msg::LShape_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<lidar_object_detection_ros2::msg::LShape_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<lidar_object_detection_ros2::msg::LShape_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      lidar_object_detection_ros2::msg::LShape_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<lidar_object_detection_ros2::msg::LShape_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      lidar_object_detection_ros2::msg::LShape_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<lidar_object_detection_ros2::msg::LShape_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<lidar_object_detection_ros2::msg::LShape_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<lidar_object_detection_ros2::msg::LShape_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__lidar_object_detection_ros2__msg__LShape
    std::shared_ptr<lidar_object_detection_ros2::msg::LShape_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__lidar_object_detection_ros2__msg__LShape
    std::shared_ptr<lidar_object_detection_ros2::msg::LShape_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const LShape_ & other) const
  {
    if (this->c1 != other.c1) {
      return false;
    }
    if (this->theta != other.theta) {
      return false;
    }
    if (this->l1 != other.l1) {
      return false;
    }
    if (this->l2 != other.l2) {
      return false;
    }
    return true;
  }
  bool operator!=(const LShape_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct LShape_

// alias to use template instance with default allocator
using LShape =
  lidar_object_detection_ros2::msg::LShape_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace lidar_object_detection_ros2

#endif  // LIDAR_OBJECT_DETECTION_ROS2__MSG__DETAIL__L_SHAPE__STRUCT_HPP_
