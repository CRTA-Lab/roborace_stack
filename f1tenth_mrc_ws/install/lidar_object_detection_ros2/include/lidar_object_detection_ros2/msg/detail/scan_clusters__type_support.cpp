// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from lidar_object_detection_ros2:msg/ScanClusters.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "lidar_object_detection_ros2/msg/detail/scan_clusters__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace lidar_object_detection_ros2
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void ScanClusters_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) lidar_object_detection_ros2::msg::ScanClusters(_init);
}

void ScanClusters_fini_function(void * message_memory)
{
  auto typed_message = static_cast<lidar_object_detection_ros2::msg::ScanClusters *>(message_memory);
  typed_message->~ScanClusters();
}

size_t size_function__ScanClusters__points(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<lidar_object_detection_ros2::msg::Pose2D> *>(untyped_member);
  return member->size();
}

const void * get_const_function__ScanClusters__points(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<lidar_object_detection_ros2::msg::Pose2D> *>(untyped_member);
  return &member[index];
}

void * get_function__ScanClusters__points(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<lidar_object_detection_ros2::msg::Pose2D> *>(untyped_member);
  return &member[index];
}

void resize_function__ScanClusters__points(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<lidar_object_detection_ros2::msg::Pose2D> *>(untyped_member);
  member->resize(size);
}

size_t size_function__ScanClusters__labels(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<int32_t> *>(untyped_member);
  return member->size();
}

const void * get_const_function__ScanClusters__labels(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<int32_t> *>(untyped_member);
  return &member[index];
}

void * get_function__ScanClusters__labels(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<int32_t> *>(untyped_member);
  return &member[index];
}

void resize_function__ScanClusters__labels(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<int32_t> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember ScanClusters_message_member_array[3] = {
  {
    "header",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Header>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(lidar_object_detection_ros2::msg::ScanClusters, header),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "points",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<lidar_object_detection_ros2::msg::Pose2D>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(lidar_object_detection_ros2::msg::ScanClusters, points),  // bytes offset in struct
    nullptr,  // default value
    size_function__ScanClusters__points,  // size() function pointer
    get_const_function__ScanClusters__points,  // get_const(index) function pointer
    get_function__ScanClusters__points,  // get(index) function pointer
    resize_function__ScanClusters__points  // resize(index) function pointer
  },
  {
    "labels",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(lidar_object_detection_ros2::msg::ScanClusters, labels),  // bytes offset in struct
    nullptr,  // default value
    size_function__ScanClusters__labels,  // size() function pointer
    get_const_function__ScanClusters__labels,  // get_const(index) function pointer
    get_function__ScanClusters__labels,  // get(index) function pointer
    resize_function__ScanClusters__labels  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers ScanClusters_message_members = {
  "lidar_object_detection_ros2::msg",  // message namespace
  "ScanClusters",  // message name
  3,  // number of fields
  sizeof(lidar_object_detection_ros2::msg::ScanClusters),
  ScanClusters_message_member_array,  // message members
  ScanClusters_init_function,  // function to initialize message memory (memory has to be allocated)
  ScanClusters_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t ScanClusters_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &ScanClusters_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace lidar_object_detection_ros2


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<lidar_object_detection_ros2::msg::ScanClusters>()
{
  return &::lidar_object_detection_ros2::msg::rosidl_typesupport_introspection_cpp::ScanClusters_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, lidar_object_detection_ros2, msg, ScanClusters)() {
  return &::lidar_object_detection_ros2::msg::rosidl_typesupport_introspection_cpp::ScanClusters_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
