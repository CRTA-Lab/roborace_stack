// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from lidar_object_detection_ros2:msg/ScanClusters.idl
// generated code does not contain a copyright notice
#include "lidar_object_detection_ros2/msg/detail/scan_clusters__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `points`
#include "lidar_object_detection_ros2/msg/detail/pose2_d__functions.h"
// Member `labels`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
lidar_object_detection_ros2__msg__ScanClusters__init(lidar_object_detection_ros2__msg__ScanClusters * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    lidar_object_detection_ros2__msg__ScanClusters__fini(msg);
    return false;
  }
  // points
  if (!lidar_object_detection_ros2__msg__Pose2D__Sequence__init(&msg->points, 0)) {
    lidar_object_detection_ros2__msg__ScanClusters__fini(msg);
    return false;
  }
  // labels
  if (!rosidl_runtime_c__int32__Sequence__init(&msg->labels, 0)) {
    lidar_object_detection_ros2__msg__ScanClusters__fini(msg);
    return false;
  }
  return true;
}

void
lidar_object_detection_ros2__msg__ScanClusters__fini(lidar_object_detection_ros2__msg__ScanClusters * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // points
  lidar_object_detection_ros2__msg__Pose2D__Sequence__fini(&msg->points);
  // labels
  rosidl_runtime_c__int32__Sequence__fini(&msg->labels);
}

bool
lidar_object_detection_ros2__msg__ScanClusters__are_equal(const lidar_object_detection_ros2__msg__ScanClusters * lhs, const lidar_object_detection_ros2__msg__ScanClusters * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // points
  if (!lidar_object_detection_ros2__msg__Pose2D__Sequence__are_equal(
      &(lhs->points), &(rhs->points)))
  {
    return false;
  }
  // labels
  if (!rosidl_runtime_c__int32__Sequence__are_equal(
      &(lhs->labels), &(rhs->labels)))
  {
    return false;
  }
  return true;
}

bool
lidar_object_detection_ros2__msg__ScanClusters__copy(
  const lidar_object_detection_ros2__msg__ScanClusters * input,
  lidar_object_detection_ros2__msg__ScanClusters * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // points
  if (!lidar_object_detection_ros2__msg__Pose2D__Sequence__copy(
      &(input->points), &(output->points)))
  {
    return false;
  }
  // labels
  if (!rosidl_runtime_c__int32__Sequence__copy(
      &(input->labels), &(output->labels)))
  {
    return false;
  }
  return true;
}

lidar_object_detection_ros2__msg__ScanClusters *
lidar_object_detection_ros2__msg__ScanClusters__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  lidar_object_detection_ros2__msg__ScanClusters * msg = (lidar_object_detection_ros2__msg__ScanClusters *)allocator.allocate(sizeof(lidar_object_detection_ros2__msg__ScanClusters), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(lidar_object_detection_ros2__msg__ScanClusters));
  bool success = lidar_object_detection_ros2__msg__ScanClusters__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
lidar_object_detection_ros2__msg__ScanClusters__destroy(lidar_object_detection_ros2__msg__ScanClusters * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    lidar_object_detection_ros2__msg__ScanClusters__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
lidar_object_detection_ros2__msg__ScanClusters__Sequence__init(lidar_object_detection_ros2__msg__ScanClusters__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  lidar_object_detection_ros2__msg__ScanClusters * data = NULL;

  if (size) {
    data = (lidar_object_detection_ros2__msg__ScanClusters *)allocator.zero_allocate(size, sizeof(lidar_object_detection_ros2__msg__ScanClusters), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = lidar_object_detection_ros2__msg__ScanClusters__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        lidar_object_detection_ros2__msg__ScanClusters__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
lidar_object_detection_ros2__msg__ScanClusters__Sequence__fini(lidar_object_detection_ros2__msg__ScanClusters__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      lidar_object_detection_ros2__msg__ScanClusters__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

lidar_object_detection_ros2__msg__ScanClusters__Sequence *
lidar_object_detection_ros2__msg__ScanClusters__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  lidar_object_detection_ros2__msg__ScanClusters__Sequence * array = (lidar_object_detection_ros2__msg__ScanClusters__Sequence *)allocator.allocate(sizeof(lidar_object_detection_ros2__msg__ScanClusters__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = lidar_object_detection_ros2__msg__ScanClusters__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
lidar_object_detection_ros2__msg__ScanClusters__Sequence__destroy(lidar_object_detection_ros2__msg__ScanClusters__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    lidar_object_detection_ros2__msg__ScanClusters__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
lidar_object_detection_ros2__msg__ScanClusters__Sequence__are_equal(const lidar_object_detection_ros2__msg__ScanClusters__Sequence * lhs, const lidar_object_detection_ros2__msg__ScanClusters__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!lidar_object_detection_ros2__msg__ScanClusters__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
lidar_object_detection_ros2__msg__ScanClusters__Sequence__copy(
  const lidar_object_detection_ros2__msg__ScanClusters__Sequence * input,
  lidar_object_detection_ros2__msg__ScanClusters__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(lidar_object_detection_ros2__msg__ScanClusters);
    lidar_object_detection_ros2__msg__ScanClusters * data =
      (lidar_object_detection_ros2__msg__ScanClusters *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!lidar_object_detection_ros2__msg__ScanClusters__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          lidar_object_detection_ros2__msg__ScanClusters__fini(&data[i]);
        }
        free(data);
        return false;
      }
    }
    output->data = data;
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!lidar_object_detection_ros2__msg__ScanClusters__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
