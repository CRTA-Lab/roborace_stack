// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from lidar_object_detection_ros2:msg/Pose2D.idl
// generated code does not contain a copyright notice

#ifndef LIDAR_OBJECT_DETECTION_ROS2__MSG__DETAIL__POSE2_D__STRUCT_H_
#define LIDAR_OBJECT_DETECTION_ROS2__MSG__DETAIL__POSE2_D__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in msg/Pose2D in the package lidar_object_detection_ros2.
typedef struct lidar_object_detection_ros2__msg__Pose2D
{
  float x;
  float y;
} lidar_object_detection_ros2__msg__Pose2D;

// Struct for a sequence of lidar_object_detection_ros2__msg__Pose2D.
typedef struct lidar_object_detection_ros2__msg__Pose2D__Sequence
{
  lidar_object_detection_ros2__msg__Pose2D * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} lidar_object_detection_ros2__msg__Pose2D__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // LIDAR_OBJECT_DETECTION_ROS2__MSG__DETAIL__POSE2_D__STRUCT_H_
