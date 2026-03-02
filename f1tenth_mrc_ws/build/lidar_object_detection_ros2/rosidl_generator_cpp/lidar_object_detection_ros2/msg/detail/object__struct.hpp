// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from lidar_object_detection_ros2:msg/Object.idl
// generated code does not contain a copyright notice

#ifndef LIDAR_OBJECT_DETECTION_ROS2__MSG__DETAIL__OBJECT__STRUCT_HPP_
#define LIDAR_OBJECT_DETECTION_ROS2__MSG__DETAIL__OBJECT__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'l_shape'
#include "lidar_object_detection_ros2/msg/detail/l_shape__struct.hpp"
// Member 'pose'
#include "lidar_object_detection_ros2/msg/detail/pose2_d__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__lidar_object_detection_ros2__msg__Object __attribute__((deprecated))
#else
# define DEPRECATED__lidar_object_detection_ros2__msg__Object __declspec(deprecated)
#endif

namespace lidar_object_detection_ros2
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Object_
{
  using Type = Object_<ContainerAllocator>;

  explicit Object_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : l_shape(_init),
    pose(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = 0l;
    }
  }

  explicit Object_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : l_shape(_alloc, _init),
    pose(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = 0l;
    }
  }

  // field types and members
  using _id_type =
    int32_t;
  _id_type id;
  using _l_shape_type =
    lidar_object_detection_ros2::msg::LShape_<ContainerAllocator>;
  _l_shape_type l_shape;
  using _pose_type =
    lidar_object_detection_ros2::msg::Pose2D_<ContainerAllocator>;
  _pose_type pose;

  // setters for named parameter idiom
  Type & set__id(
    const int32_t & _arg)
  {
    this->id = _arg;
    return *this;
  }
  Type & set__l_shape(
    const lidar_object_detection_ros2::msg::LShape_<ContainerAllocator> & _arg)
  {
    this->l_shape = _arg;
    return *this;
  }
  Type & set__pose(
    const lidar_object_detection_ros2::msg::Pose2D_<ContainerAllocator> & _arg)
  {
    this->pose = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    lidar_object_detection_ros2::msg::Object_<ContainerAllocator> *;
  using ConstRawPtr =
    const lidar_object_detection_ros2::msg::Object_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<lidar_object_detection_ros2::msg::Object_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<lidar_object_detection_ros2::msg::Object_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      lidar_object_detection_ros2::msg::Object_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<lidar_object_detection_ros2::msg::Object_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      lidar_object_detection_ros2::msg::Object_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<lidar_object_detection_ros2::msg::Object_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<lidar_object_detection_ros2::msg::Object_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<lidar_object_detection_ros2::msg::Object_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__lidar_object_detection_ros2__msg__Object
    std::shared_ptr<lidar_object_detection_ros2::msg::Object_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__lidar_object_detection_ros2__msg__Object
    std::shared_ptr<lidar_object_detection_ros2::msg::Object_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Object_ & other) const
  {
    if (this->id != other.id) {
      return false;
    }
    if (this->l_shape != other.l_shape) {
      return false;
    }
    if (this->pose != other.pose) {
      return false;
    }
    return true;
  }
  bool operator!=(const Object_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Object_

// alias to use template instance with default allocator
using Object =
  lidar_object_detection_ros2::msg::Object_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace lidar_object_detection_ros2

#endif  // LIDAR_OBJECT_DETECTION_ROS2__MSG__DETAIL__OBJECT__STRUCT_HPP_
