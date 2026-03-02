// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from lidar_object_detection_ros2:msg/LShape.idl
// generated code does not contain a copyright notice
#include "lidar_object_detection_ros2/msg/detail/l_shape__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "lidar_object_detection_ros2/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "lidar_object_detection_ros2/msg/detail/l_shape__struct.h"
#include "lidar_object_detection_ros2/msg/detail/l_shape__functions.h"
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

#include "lidar_object_detection_ros2/msg/detail/pose2_d__functions.h"  // c1

// forward declare type support functions
size_t get_serialized_size_lidar_object_detection_ros2__msg__Pose2D(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_lidar_object_detection_ros2__msg__Pose2D(
  bool & full_bounded,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, lidar_object_detection_ros2, msg, Pose2D)();


using _LShape__ros_msg_type = lidar_object_detection_ros2__msg__LShape;

static bool _LShape__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _LShape__ros_msg_type * ros_message = static_cast<const _LShape__ros_msg_type *>(untyped_ros_message);
  // Field name: c1
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, lidar_object_detection_ros2, msg, Pose2D
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->c1, cdr))
    {
      return false;
    }
  }

  // Field name: theta
  {
    cdr << ros_message->theta;
  }

  // Field name: l1
  {
    cdr << ros_message->l1;
  }

  // Field name: l2
  {
    cdr << ros_message->l2;
  }

  return true;
}

static bool _LShape__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _LShape__ros_msg_type * ros_message = static_cast<_LShape__ros_msg_type *>(untyped_ros_message);
  // Field name: c1
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, lidar_object_detection_ros2, msg, Pose2D
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->c1))
    {
      return false;
    }
  }

  // Field name: theta
  {
    cdr >> ros_message->theta;
  }

  // Field name: l1
  {
    cdr >> ros_message->l1;
  }

  // Field name: l2
  {
    cdr >> ros_message->l2;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_lidar_object_detection_ros2
size_t get_serialized_size_lidar_object_detection_ros2__msg__LShape(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _LShape__ros_msg_type * ros_message = static_cast<const _LShape__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name c1

  current_alignment += get_serialized_size_lidar_object_detection_ros2__msg__Pose2D(
    &(ros_message->c1), current_alignment);
  // field.name theta
  {
    size_t item_size = sizeof(ros_message->theta);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name l1
  {
    size_t item_size = sizeof(ros_message->l1);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name l2
  {
    size_t item_size = sizeof(ros_message->l2);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _LShape__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_lidar_object_detection_ros2__msg__LShape(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_lidar_object_detection_ros2
size_t max_serialized_size_lidar_object_detection_ros2__msg__LShape(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;

  // member: c1
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        max_serialized_size_lidar_object_detection_ros2__msg__Pose2D(
        full_bounded, current_alignment);
    }
  }
  // member: theta
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: l1
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: l2
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  return current_alignment - initial_alignment;
}

static size_t _LShape__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_lidar_object_detection_ros2__msg__LShape(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_LShape = {
  "lidar_object_detection_ros2::msg",
  "LShape",
  _LShape__cdr_serialize,
  _LShape__cdr_deserialize,
  _LShape__get_serialized_size,
  _LShape__max_serialized_size
};

static rosidl_message_type_support_t _LShape__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_LShape,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, lidar_object_detection_ros2, msg, LShape)() {
  return &_LShape__type_support;
}

#if defined(__cplusplus)
}
#endif
