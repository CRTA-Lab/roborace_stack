// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from realsense2_camera_msgs:srv/CalibConfigWrite.idl
// generated code does not contain a copyright notice

#ifndef REALSENSE2_CAMERA_MSGS__SRV__DETAIL__CALIB_CONFIG_WRITE__TRAITS_HPP_
#define REALSENSE2_CAMERA_MSGS__SRV__DETAIL__CALIB_CONFIG_WRITE__TRAITS_HPP_

#include "realsense2_camera_msgs/srv/detail/calib_config_write__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<realsense2_camera_msgs::srv::CalibConfigWrite_Request>()
{
  return "realsense2_camera_msgs::srv::CalibConfigWrite_Request";
}

template<>
inline const char * name<realsense2_camera_msgs::srv::CalibConfigWrite_Request>()
{
  return "realsense2_camera_msgs/srv/CalibConfigWrite_Request";
}

template<>
struct has_fixed_size<realsense2_camera_msgs::srv::CalibConfigWrite_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<realsense2_camera_msgs::srv::CalibConfigWrite_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<realsense2_camera_msgs::srv::CalibConfigWrite_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<realsense2_camera_msgs::srv::CalibConfigWrite_Response>()
{
  return "realsense2_camera_msgs::srv::CalibConfigWrite_Response";
}

template<>
inline const char * name<realsense2_camera_msgs::srv::CalibConfigWrite_Response>()
{
  return "realsense2_camera_msgs/srv/CalibConfigWrite_Response";
}

template<>
struct has_fixed_size<realsense2_camera_msgs::srv::CalibConfigWrite_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<realsense2_camera_msgs::srv::CalibConfigWrite_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<realsense2_camera_msgs::srv::CalibConfigWrite_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<realsense2_camera_msgs::srv::CalibConfigWrite>()
{
  return "realsense2_camera_msgs::srv::CalibConfigWrite";
}

template<>
inline const char * name<realsense2_camera_msgs::srv::CalibConfigWrite>()
{
  return "realsense2_camera_msgs/srv/CalibConfigWrite";
}

template<>
struct has_fixed_size<realsense2_camera_msgs::srv::CalibConfigWrite>
  : std::integral_constant<
    bool,
    has_fixed_size<realsense2_camera_msgs::srv::CalibConfigWrite_Request>::value &&
    has_fixed_size<realsense2_camera_msgs::srv::CalibConfigWrite_Response>::value
  >
{
};

template<>
struct has_bounded_size<realsense2_camera_msgs::srv::CalibConfigWrite>
  : std::integral_constant<
    bool,
    has_bounded_size<realsense2_camera_msgs::srv::CalibConfigWrite_Request>::value &&
    has_bounded_size<realsense2_camera_msgs::srv::CalibConfigWrite_Response>::value
  >
{
};

template<>
struct is_service<realsense2_camera_msgs::srv::CalibConfigWrite>
  : std::true_type
{
};

template<>
struct is_service_request<realsense2_camera_msgs::srv::CalibConfigWrite_Request>
  : std::true_type
{
};

template<>
struct is_service_response<realsense2_camera_msgs::srv::CalibConfigWrite_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // REALSENSE2_CAMERA_MSGS__SRV__DETAIL__CALIB_CONFIG_WRITE__TRAITS_HPP_
