// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from lidar_object_detection_ros2:msg/ScanClusters.idl
// generated code does not contain a copyright notice

#ifndef LIDAR_OBJECT_DETECTION_ROS2__MSG__DETAIL__SCAN_CLUSTERS__STRUCT_HPP_
#define LIDAR_OBJECT_DETECTION_ROS2__MSG__DETAIL__SCAN_CLUSTERS__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"
// Member 'points'
#include "lidar_object_detection_ros2/msg/detail/pose2_d__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__lidar_object_detection_ros2__msg__ScanClusters __attribute__((deprecated))
#else
# define DEPRECATED__lidar_object_detection_ros2__msg__ScanClusters __declspec(deprecated)
#endif

namespace lidar_object_detection_ros2
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ScanClusters_
{
  using Type = ScanClusters_<ContainerAllocator>;

  explicit ScanClusters_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    (void)_init;
  }

  explicit ScanClusters_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _points_type =
    std::vector<lidar_object_detection_ros2::msg::Pose2D_<ContainerAllocator>, typename ContainerAllocator::template rebind<lidar_object_detection_ros2::msg::Pose2D_<ContainerAllocator>>::other>;
  _points_type points;
  using _labels_type =
    std::vector<int32_t, typename ContainerAllocator::template rebind<int32_t>::other>;
  _labels_type labels;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__points(
    const std::vector<lidar_object_detection_ros2::msg::Pose2D_<ContainerAllocator>, typename ContainerAllocator::template rebind<lidar_object_detection_ros2::msg::Pose2D_<ContainerAllocator>>::other> & _arg)
  {
    this->points = _arg;
    return *this;
  }
  Type & set__labels(
    const std::vector<int32_t, typename ContainerAllocator::template rebind<int32_t>::other> & _arg)
  {
    this->labels = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    lidar_object_detection_ros2::msg::ScanClusters_<ContainerAllocator> *;
  using ConstRawPtr =
    const lidar_object_detection_ros2::msg::ScanClusters_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<lidar_object_detection_ros2::msg::ScanClusters_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<lidar_object_detection_ros2::msg::ScanClusters_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      lidar_object_detection_ros2::msg::ScanClusters_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<lidar_object_detection_ros2::msg::ScanClusters_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      lidar_object_detection_ros2::msg::ScanClusters_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<lidar_object_detection_ros2::msg::ScanClusters_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<lidar_object_detection_ros2::msg::ScanClusters_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<lidar_object_detection_ros2::msg::ScanClusters_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__lidar_object_detection_ros2__msg__ScanClusters
    std::shared_ptr<lidar_object_detection_ros2::msg::ScanClusters_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__lidar_object_detection_ros2__msg__ScanClusters
    std::shared_ptr<lidar_object_detection_ros2::msg::ScanClusters_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ScanClusters_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->points != other.points) {
      return false;
    }
    if (this->labels != other.labels) {
      return false;
    }
    return true;
  }
  bool operator!=(const ScanClusters_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ScanClusters_

// alias to use template instance with default allocator
using ScanClusters =
  lidar_object_detection_ros2::msg::ScanClusters_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace lidar_object_detection_ros2

#endif  // LIDAR_OBJECT_DETECTION_ROS2__MSG__DETAIL__SCAN_CLUSTERS__STRUCT_HPP_
