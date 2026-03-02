// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from lidar_object_detection_ros2:msg/ObjectsArray.idl
// generated code does not contain a copyright notice

#ifndef LIDAR_OBJECT_DETECTION_ROS2__MSG__DETAIL__OBJECTS_ARRAY__STRUCT_H_
#define LIDAR_OBJECT_DETECTION_ROS2__MSG__DETAIL__OBJECTS_ARRAY__STRUCT_H_

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
// Member 'objects'
#include "lidar_object_detection_ros2/msg/detail/object__struct.h"

// Struct defined in msg/ObjectsArray in the package lidar_object_detection_ros2.
typedef struct lidar_object_detection_ros2__msg__ObjectsArray
{
  std_msgs__msg__Header header;
  lidar_object_detection_ros2__msg__Object__Sequence objects;
} lidar_object_detection_ros2__msg__ObjectsArray;

// Struct for a sequence of lidar_object_detection_ros2__msg__ObjectsArray.
typedef struct lidar_object_detection_ros2__msg__ObjectsArray__Sequence
{
  lidar_object_detection_ros2__msg__ObjectsArray * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} lidar_object_detection_ros2__msg__ObjectsArray__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // LIDAR_OBJECT_DETECTION_ROS2__MSG__DETAIL__OBJECTS_ARRAY__STRUCT_H_
