// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from lidar_object_detection_ros2:msg/LShape.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "lidar_object_detection_ros2/msg/detail/l_shape__rosidl_typesupport_introspection_c.h"
#include "lidar_object_detection_ros2/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "lidar_object_detection_ros2/msg/detail/l_shape__functions.h"
#include "lidar_object_detection_ros2/msg/detail/l_shape__struct.h"


// Include directives for member types
// Member `c1`
#include "lidar_object_detection_ros2/msg/pose2_d.h"
// Member `c1`
#include "lidar_object_detection_ros2/msg/detail/pose2_d__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void LShape__rosidl_typesupport_introspection_c__LShape_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  lidar_object_detection_ros2__msg__LShape__init(message_memory);
}

void LShape__rosidl_typesupport_introspection_c__LShape_fini_function(void * message_memory)
{
  lidar_object_detection_ros2__msg__LShape__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember LShape__rosidl_typesupport_introspection_c__LShape_message_member_array[4] = {
  {
    "c1",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(lidar_object_detection_ros2__msg__LShape, c1),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "theta",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(lidar_object_detection_ros2__msg__LShape, theta),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "l1",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(lidar_object_detection_ros2__msg__LShape, l1),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "l2",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(lidar_object_detection_ros2__msg__LShape, l2),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers LShape__rosidl_typesupport_introspection_c__LShape_message_members = {
  "lidar_object_detection_ros2__msg",  // message namespace
  "LShape",  // message name
  4,  // number of fields
  sizeof(lidar_object_detection_ros2__msg__LShape),
  LShape__rosidl_typesupport_introspection_c__LShape_message_member_array,  // message members
  LShape__rosidl_typesupport_introspection_c__LShape_init_function,  // function to initialize message memory (memory has to be allocated)
  LShape__rosidl_typesupport_introspection_c__LShape_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t LShape__rosidl_typesupport_introspection_c__LShape_message_type_support_handle = {
  0,
  &LShape__rosidl_typesupport_introspection_c__LShape_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_lidar_object_detection_ros2
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, lidar_object_detection_ros2, msg, LShape)() {
  LShape__rosidl_typesupport_introspection_c__LShape_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, lidar_object_detection_ros2, msg, Pose2D)();
  if (!LShape__rosidl_typesupport_introspection_c__LShape_message_type_support_handle.typesupport_identifier) {
    LShape__rosidl_typesupport_introspection_c__LShape_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &LShape__rosidl_typesupport_introspection_c__LShape_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
