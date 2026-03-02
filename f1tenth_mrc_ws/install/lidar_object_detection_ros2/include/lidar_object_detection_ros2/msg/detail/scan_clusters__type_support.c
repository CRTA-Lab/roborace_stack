// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from lidar_object_detection_ros2:msg/ScanClusters.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "lidar_object_detection_ros2/msg/detail/scan_clusters__rosidl_typesupport_introspection_c.h"
#include "lidar_object_detection_ros2/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "lidar_object_detection_ros2/msg/detail/scan_clusters__functions.h"
#include "lidar_object_detection_ros2/msg/detail/scan_clusters__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `points`
#include "lidar_object_detection_ros2/msg/pose2_d.h"
// Member `points`
#include "lidar_object_detection_ros2/msg/detail/pose2_d__rosidl_typesupport_introspection_c.h"
// Member `labels`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void ScanClusters__rosidl_typesupport_introspection_c__ScanClusters_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  lidar_object_detection_ros2__msg__ScanClusters__init(message_memory);
}

void ScanClusters__rosidl_typesupport_introspection_c__ScanClusters_fini_function(void * message_memory)
{
  lidar_object_detection_ros2__msg__ScanClusters__fini(message_memory);
}

size_t ScanClusters__rosidl_typesupport_introspection_c__size_function__Pose2D__points(
  const void * untyped_member)
{
  const lidar_object_detection_ros2__msg__Pose2D__Sequence * member =
    (const lidar_object_detection_ros2__msg__Pose2D__Sequence *)(untyped_member);
  return member->size;
}

const void * ScanClusters__rosidl_typesupport_introspection_c__get_const_function__Pose2D__points(
  const void * untyped_member, size_t index)
{
  const lidar_object_detection_ros2__msg__Pose2D__Sequence * member =
    (const lidar_object_detection_ros2__msg__Pose2D__Sequence *)(untyped_member);
  return &member->data[index];
}

void * ScanClusters__rosidl_typesupport_introspection_c__get_function__Pose2D__points(
  void * untyped_member, size_t index)
{
  lidar_object_detection_ros2__msg__Pose2D__Sequence * member =
    (lidar_object_detection_ros2__msg__Pose2D__Sequence *)(untyped_member);
  return &member->data[index];
}

bool ScanClusters__rosidl_typesupport_introspection_c__resize_function__Pose2D__points(
  void * untyped_member, size_t size)
{
  lidar_object_detection_ros2__msg__Pose2D__Sequence * member =
    (lidar_object_detection_ros2__msg__Pose2D__Sequence *)(untyped_member);
  lidar_object_detection_ros2__msg__Pose2D__Sequence__fini(member);
  return lidar_object_detection_ros2__msg__Pose2D__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember ScanClusters__rosidl_typesupport_introspection_c__ScanClusters_message_member_array[3] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(lidar_object_detection_ros2__msg__ScanClusters, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "points",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(lidar_object_detection_ros2__msg__ScanClusters, points),  // bytes offset in struct
    NULL,  // default value
    ScanClusters__rosidl_typesupport_introspection_c__size_function__Pose2D__points,  // size() function pointer
    ScanClusters__rosidl_typesupport_introspection_c__get_const_function__Pose2D__points,  // get_const(index) function pointer
    ScanClusters__rosidl_typesupport_introspection_c__get_function__Pose2D__points,  // get(index) function pointer
    ScanClusters__rosidl_typesupport_introspection_c__resize_function__Pose2D__points  // resize(index) function pointer
  },
  {
    "labels",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(lidar_object_detection_ros2__msg__ScanClusters, labels),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers ScanClusters__rosidl_typesupport_introspection_c__ScanClusters_message_members = {
  "lidar_object_detection_ros2__msg",  // message namespace
  "ScanClusters",  // message name
  3,  // number of fields
  sizeof(lidar_object_detection_ros2__msg__ScanClusters),
  ScanClusters__rosidl_typesupport_introspection_c__ScanClusters_message_member_array,  // message members
  ScanClusters__rosidl_typesupport_introspection_c__ScanClusters_init_function,  // function to initialize message memory (memory has to be allocated)
  ScanClusters__rosidl_typesupport_introspection_c__ScanClusters_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t ScanClusters__rosidl_typesupport_introspection_c__ScanClusters_message_type_support_handle = {
  0,
  &ScanClusters__rosidl_typesupport_introspection_c__ScanClusters_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_lidar_object_detection_ros2
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, lidar_object_detection_ros2, msg, ScanClusters)() {
  ScanClusters__rosidl_typesupport_introspection_c__ScanClusters_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  ScanClusters__rosidl_typesupport_introspection_c__ScanClusters_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, lidar_object_detection_ros2, msg, Pose2D)();
  if (!ScanClusters__rosidl_typesupport_introspection_c__ScanClusters_message_type_support_handle.typesupport_identifier) {
    ScanClusters__rosidl_typesupport_introspection_c__ScanClusters_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &ScanClusters__rosidl_typesupport_introspection_c__ScanClusters_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
