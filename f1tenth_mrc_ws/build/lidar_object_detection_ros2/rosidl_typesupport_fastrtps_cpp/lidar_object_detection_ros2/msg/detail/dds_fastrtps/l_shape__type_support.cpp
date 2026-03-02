// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from lidar_object_detection_ros2:msg/LShape.idl
// generated code does not contain a copyright notice
#include "lidar_object_detection_ros2/msg/detail/l_shape__rosidl_typesupport_fastrtps_cpp.hpp"
#include "lidar_object_detection_ros2/msg/detail/l_shape__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions
namespace lidar_object_detection_ros2
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const lidar_object_detection_ros2::msg::Pose2D &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  lidar_object_detection_ros2::msg::Pose2D &);
size_t get_serialized_size(
  const lidar_object_detection_ros2::msg::Pose2D &,
  size_t current_alignment);
size_t
max_serialized_size_Pose2D(
  bool & full_bounded,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace lidar_object_detection_ros2


namespace lidar_object_detection_ros2
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_lidar_object_detection_ros2
cdr_serialize(
  const lidar_object_detection_ros2::msg::LShape & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: c1
  lidar_object_detection_ros2::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.c1,
    cdr);
  // Member: theta
  cdr << ros_message.theta;
  // Member: l1
  cdr << ros_message.l1;
  // Member: l2
  cdr << ros_message.l2;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_lidar_object_detection_ros2
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  lidar_object_detection_ros2::msg::LShape & ros_message)
{
  // Member: c1
  lidar_object_detection_ros2::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.c1);

  // Member: theta
  cdr >> ros_message.theta;

  // Member: l1
  cdr >> ros_message.l1;

  // Member: l2
  cdr >> ros_message.l2;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_lidar_object_detection_ros2
get_serialized_size(
  const lidar_object_detection_ros2::msg::LShape & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: c1

  current_alignment +=
    lidar_object_detection_ros2::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.c1, current_alignment);
  // Member: theta
  {
    size_t item_size = sizeof(ros_message.theta);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: l1
  {
    size_t item_size = sizeof(ros_message.l1);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: l2
  {
    size_t item_size = sizeof(ros_message.l2);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_lidar_object_detection_ros2
max_serialized_size_LShape(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;


  // Member: c1
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        lidar_object_detection_ros2::msg::typesupport_fastrtps_cpp::max_serialized_size_Pose2D(
        full_bounded, current_alignment);
    }
  }

  // Member: theta
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: l1
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: l2
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  return current_alignment - initial_alignment;
}

static bool _LShape__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const lidar_object_detection_ros2::msg::LShape *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _LShape__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<lidar_object_detection_ros2::msg::LShape *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _LShape__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const lidar_object_detection_ros2::msg::LShape *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _LShape__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_LShape(full_bounded, 0);
}

static message_type_support_callbacks_t _LShape__callbacks = {
  "lidar_object_detection_ros2::msg",
  "LShape",
  _LShape__cdr_serialize,
  _LShape__cdr_deserialize,
  _LShape__get_serialized_size,
  _LShape__max_serialized_size
};

static rosidl_message_type_support_t _LShape__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_LShape__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace lidar_object_detection_ros2

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_lidar_object_detection_ros2
const rosidl_message_type_support_t *
get_message_type_support_handle<lidar_object_detection_ros2::msg::LShape>()
{
  return &lidar_object_detection_ros2::msg::typesupport_fastrtps_cpp::_LShape__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, lidar_object_detection_ros2, msg, LShape)() {
  return &lidar_object_detection_ros2::msg::typesupport_fastrtps_cpp::_LShape__handle;
}

#ifdef __cplusplus
}
#endif
