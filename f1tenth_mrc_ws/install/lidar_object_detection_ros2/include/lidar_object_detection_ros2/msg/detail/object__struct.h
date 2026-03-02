// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from lidar_object_detection_ros2:msg/Object.idl
// generated code does not contain a copyright notice

#ifndef LIDAR_OBJECT_DETECTION_ROS2__MSG__DETAIL__OBJECT__STRUCT_H_
#define LIDAR_OBJECT_DETECTION_ROS2__MSG__DETAIL__OBJECT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'l_shape'
#include "lidar_object_detection_ros2/msg/detail/l_shape__struct.h"
// Member 'pose'
#include "lidar_object_detection_ros2/msg/detail/pose2_d__struct.h"

// Struct defined in msg/Object in the package lidar_object_detection_ros2.
typedef struct lidar_object_detection_ros2__msg__Object
{
  int32_t id;
  lidar_object_detection_ros2__msg__LShape l_shape;
  lidar_object_detection_ros2__msg__Pose2D pose;
} lidar_object_detection_ros2__msg__Object;

// Struct for a sequence of lidar_object_detection_ros2__msg__Object.
typedef struct lidar_object_detection_ros2__msg__Object__Sequence
{
  lidar_object_detection_ros2__msg__Object * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} lidar_object_detection_ros2__msg__Object__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // LIDAR_OBJECT_DETECTION_ROS2__MSG__DETAIL__OBJECT__STRUCT_H_
