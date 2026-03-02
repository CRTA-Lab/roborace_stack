// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from lidar_object_detection_ros2:msg/ScanClusters.idl
// generated code does not contain a copyright notice

#ifndef LIDAR_OBJECT_DETECTION_ROS2__MSG__DETAIL__SCAN_CLUSTERS__STRUCT_H_
#define LIDAR_OBJECT_DETECTION_ROS2__MSG__DETAIL__SCAN_CLUSTERS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'points'
#include "lidar_object_detection_ros2/msg/detail/pose2_d__struct.h"
// Member 'labels'
#include "rosidl_runtime_c/primitives_sequence.h"

// Struct defined in msg/ScanClusters in the package lidar_object_detection_ros2.
typedef struct lidar_object_detection_ros2__msg__ScanClusters
{
  std_msgs__msg__Header header;
  lidar_object_detection_ros2__msg__Pose2D__Sequence points;
  rosidl_runtime_c__int32__Sequence labels;
} lidar_object_detection_ros2__msg__ScanClusters;

// Struct for a sequence of lidar_object_detection_ros2__msg__ScanClusters.
typedef struct lidar_object_detection_ros2__msg__ScanClusters__Sequence
{
  lidar_object_detection_ros2__msg__ScanClusters * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} lidar_object_detection_ros2__msg__ScanClusters__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // LIDAR_OBJECT_DETECTION_ROS2__MSG__DETAIL__SCAN_CLUSTERS__STRUCT_H_
