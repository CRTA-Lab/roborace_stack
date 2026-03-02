// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from lidar_object_detection_ros2:msg/LShape.idl
// generated code does not contain a copyright notice

#ifndef LIDAR_OBJECT_DETECTION_ROS2__MSG__DETAIL__L_SHAPE__STRUCT_H_
#define LIDAR_OBJECT_DETECTION_ROS2__MSG__DETAIL__L_SHAPE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'c1'
#include "lidar_object_detection_ros2/msg/detail/pose2_d__struct.h"

// Struct defined in msg/LShape in the package lidar_object_detection_ros2.
typedef struct lidar_object_detection_ros2__msg__LShape
{
  lidar_object_detection_ros2__msg__Pose2D c1;
  float theta;
  float l1;
  float l2;
} lidar_object_detection_ros2__msg__LShape;

// Struct for a sequence of lidar_object_detection_ros2__msg__LShape.
typedef struct lidar_object_detection_ros2__msg__LShape__Sequence
{
  lidar_object_detection_ros2__msg__LShape * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} lidar_object_detection_ros2__msg__LShape__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // LIDAR_OBJECT_DETECTION_ROS2__MSG__DETAIL__L_SHAPE__STRUCT_H_
