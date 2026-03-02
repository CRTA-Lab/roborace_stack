// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from lidar_object_detection_ros2:msg/Pose2D.idl
// generated code does not contain a copyright notice
#include "lidar_object_detection_ros2/msg/detail/pose2_d__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
lidar_object_detection_ros2__msg__Pose2D__init(lidar_object_detection_ros2__msg__Pose2D * msg)
{
  if (!msg) {
    return false;
  }
  // x
  // y
  return true;
}

void
lidar_object_detection_ros2__msg__Pose2D__fini(lidar_object_detection_ros2__msg__Pose2D * msg)
{
  if (!msg) {
    return;
  }
  // x
  // y
}

bool
lidar_object_detection_ros2__msg__Pose2D__are_equal(const lidar_object_detection_ros2__msg__Pose2D * lhs, const lidar_object_detection_ros2__msg__Pose2D * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // x
  if (lhs->x != rhs->x) {
    return false;
  }
  // y
  if (lhs->y != rhs->y) {
    return false;
  }
  return true;
}

bool
lidar_object_detection_ros2__msg__Pose2D__copy(
  const lidar_object_detection_ros2__msg__Pose2D * input,
  lidar_object_detection_ros2__msg__Pose2D * output)
{
  if (!input || !output) {
    return false;
  }
  // x
  output->x = input->x;
  // y
  output->y = input->y;
  return true;
}

lidar_object_detection_ros2__msg__Pose2D *
lidar_object_detection_ros2__msg__Pose2D__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  lidar_object_detection_ros2__msg__Pose2D * msg = (lidar_object_detection_ros2__msg__Pose2D *)allocator.allocate(sizeof(lidar_object_detection_ros2__msg__Pose2D), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(lidar_object_detection_ros2__msg__Pose2D));
  bool success = lidar_object_detection_ros2__msg__Pose2D__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
lidar_object_detection_ros2__msg__Pose2D__destroy(lidar_object_detection_ros2__msg__Pose2D * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    lidar_object_detection_ros2__msg__Pose2D__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
lidar_object_detection_ros2__msg__Pose2D__Sequence__init(lidar_object_detection_ros2__msg__Pose2D__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  lidar_object_detection_ros2__msg__Pose2D * data = NULL;

  if (size) {
    data = (lidar_object_detection_ros2__msg__Pose2D *)allocator.zero_allocate(size, sizeof(lidar_object_detection_ros2__msg__Pose2D), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = lidar_object_detection_ros2__msg__Pose2D__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        lidar_object_detection_ros2__msg__Pose2D__fini(&data[i - 1]);
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
lidar_object_detection_ros2__msg__Pose2D__Sequence__fini(lidar_object_detection_ros2__msg__Pose2D__Sequence * array)
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
      lidar_object_detection_ros2__msg__Pose2D__fini(&array->data[i]);
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

lidar_object_detection_ros2__msg__Pose2D__Sequence *
lidar_object_detection_ros2__msg__Pose2D__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  lidar_object_detection_ros2__msg__Pose2D__Sequence * array = (lidar_object_detection_ros2__msg__Pose2D__Sequence *)allocator.allocate(sizeof(lidar_object_detection_ros2__msg__Pose2D__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = lidar_object_detection_ros2__msg__Pose2D__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
lidar_object_detection_ros2__msg__Pose2D__Sequence__destroy(lidar_object_detection_ros2__msg__Pose2D__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    lidar_object_detection_ros2__msg__Pose2D__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
lidar_object_detection_ros2__msg__Pose2D__Sequence__are_equal(const lidar_object_detection_ros2__msg__Pose2D__Sequence * lhs, const lidar_object_detection_ros2__msg__Pose2D__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!lidar_object_detection_ros2__msg__Pose2D__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
lidar_object_detection_ros2__msg__Pose2D__Sequence__copy(
  const lidar_object_detection_ros2__msg__Pose2D__Sequence * input,
  lidar_object_detection_ros2__msg__Pose2D__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(lidar_object_detection_ros2__msg__Pose2D);
    lidar_object_detection_ros2__msg__Pose2D * data =
      (lidar_object_detection_ros2__msg__Pose2D *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!lidar_object_detection_ros2__msg__Pose2D__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          lidar_object_detection_ros2__msg__Pose2D__fini(&data[i]);
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
    if (!lidar_object_detection_ros2__msg__Pose2D__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
