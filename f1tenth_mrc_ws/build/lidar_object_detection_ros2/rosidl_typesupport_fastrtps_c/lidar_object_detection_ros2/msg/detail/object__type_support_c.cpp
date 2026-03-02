// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from lidar_object_detection_ros2:msg/Object.idl
// generated code does not contain a copyright notice
#include "lidar_object_detection_ros2/msg/detail/object__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "lidar_object_detection_ros2/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "lidar_object_detection_ros2/msg/detail/object__struct.h"
#include "lidar_object_detection_ros2/msg/detail/object__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "lidar_object_detection_ros2/msg/detail/l_shape__functions.h"  // l_shape
#include "lidar_object_detection_ros2/msg/detail/pose2_d__functions.h"  // pose

// forward declare type support functions
size_t get_serialized_size_lidar_object_detection_ros2__msg__LShape(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_lidar_object_detection_ros2__msg__LShape(
  bool & full_bounded,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, lidar_object_detection_ros2, msg, LShape)();
size_t get_serialized_size_lidar_object_detection_ros2__msg__Pose2D(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_lidar_object_detection_ros2__msg__Pose2D(
  bool & full_bounded,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, lidar_object_detection_ros2, msg, Pose2D)();


using _Object__ros_msg_type = lidar_object_detection_ros2__msg__Object;

static bool _Object__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _Object__ros_msg_type * ros_message = static_cast<const _Object__ros_msg_type *>(untyped_ros_message);
  // Field name: id
  {
    cdr << ros_message->id;
  }

  // Field name: l_shape
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, lidar_object_detection_ros2, msg, LShape
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->l_shape, cdr))
    {
      return false;
    }
  }

  // Field name: pose
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, lidar_object_detection_ros2, msg, Pose2D
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->pose, cdr))
    {
      return false;
    }
  }

  return true;
}

static bool _Object__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _Object__ros_msg_type * ros_message = static_cast<_Object__ros_msg_type *>(untyped_ros_message);
  // Field name: id
  {
    cdr >> ros_message->id;
  }

  // Field name: l_shape
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, lidar_object_detection_ros2, msg, LShape
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->l_shape))
    {
      return false;
    }
  }

  // Field name: pose
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, lidar_object_detection_ros2, msg, Pose2D
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->pose))
    {
      return false;
    }
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_lidar_object_detection_ros2
size_t get_serialized_size_lidar_object_detection_ros2__msg__Object(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _Object__ros_msg_type * ros_message = static_cast<const _Object__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name id
  {
    size_t item_size = sizeof(ros_message->id);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name l_shape

  current_alignment += get_serialized_size_lidar_object_detection_ros2__msg__LShape(
    &(ros_message->l_shape), current_alignment);
  // field.name pose

  current_alignment += get_serialized_size_lidar_object_detection_ros2__msg__Pose2D(
    &(ros_message->pose), current_alignment);

  return current_alignment - initial_alignment;
}

static uint32_t _Object__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_lidar_object_detection_ros2__msg__Object(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_lidar_object_detection_ros2
size_t max_serialized_size_lidar_object_detection_ros2__msg__Object(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;

  // member: id
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: l_shape
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        max_serialized_size_lidar_object_detection_ros2__msg__LShape(
        full_bounded, current_alignment);
    }
  }
  // member: pose
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        max_serialized_size_lidar_object_detection_ros2__msg__Pose2D(
        full_bounded, current_alignment);
    }
  }

  return current_alignment - initial_alignment;
}

static size_t _Object__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_lidar_object_detection_ros2__msg__Object(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_Object = {
  "lidar_object_detection_ros2::msg",
  "Object",
  _Object__cdr_serialize,
  _Object__cdr_deserialize,
  _Object__get_serialized_size,
  _Object__max_serialized_size
};

static rosidl_message_type_support_t _Object__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_Object,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, lidar_object_detection_ros2, msg, Object)() {
  return &_Object__type_support;
}

#if defined(__cplusplus)
}
#endif
