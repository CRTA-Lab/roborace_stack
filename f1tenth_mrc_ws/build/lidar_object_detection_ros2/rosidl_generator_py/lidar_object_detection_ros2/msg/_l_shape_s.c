// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from lidar_object_detection_ros2:msg/LShape.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "lidar_object_detection_ros2/msg/detail/l_shape__struct.h"
#include "lidar_object_detection_ros2/msg/detail/l_shape__functions.h"

bool lidar_object_detection_ros2__msg__pose2_d__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * lidar_object_detection_ros2__msg__pose2_d__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool lidar_object_detection_ros2__msg__l_shape__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[48];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("lidar_object_detection_ros2.msg._l_shape.LShape", full_classname_dest, 47) == 0);
  }
  lidar_object_detection_ros2__msg__LShape * ros_message = _ros_message;
  {  // c1
    PyObject * field = PyObject_GetAttrString(_pymsg, "c1");
    if (!field) {
      return false;
    }
    if (!lidar_object_detection_ros2__msg__pose2_d__convert_from_py(field, &ros_message->c1)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // theta
    PyObject * field = PyObject_GetAttrString(_pymsg, "theta");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->theta = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // l1
    PyObject * field = PyObject_GetAttrString(_pymsg, "l1");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->l1 = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // l2
    PyObject * field = PyObject_GetAttrString(_pymsg, "l2");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->l2 = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * lidar_object_detection_ros2__msg__l_shape__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of LShape */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("lidar_object_detection_ros2.msg._l_shape");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "LShape");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  lidar_object_detection_ros2__msg__LShape * ros_message = (lidar_object_detection_ros2__msg__LShape *)raw_ros_message;
  {  // c1
    PyObject * field = NULL;
    field = lidar_object_detection_ros2__msg__pose2_d__convert_to_py(&ros_message->c1);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "c1", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // theta
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->theta);
    {
      int rc = PyObject_SetAttrString(_pymessage, "theta", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // l1
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->l1);
    {
      int rc = PyObject_SetAttrString(_pymessage, "l1", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // l2
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->l2);
    {
      int rc = PyObject_SetAttrString(_pymessage, "l2", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
