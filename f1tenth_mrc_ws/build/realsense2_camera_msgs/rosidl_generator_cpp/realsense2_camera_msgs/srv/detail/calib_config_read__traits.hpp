// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from realsense2_camera_msgs:srv/CalibConfigRead.idl
// generated code does not contain a copyright notice

#ifndef REALSENSE2_CAMERA_MSGS__SRV__DETAIL__CALIB_CONFIG_READ__TRAITS_HPP_
#define REALSENSE2_CAMERA_MSGS__SRV__DETAIL__CALIB_CONFIG_READ__TRAITS_HPP_

#include "realsense2_camera_msgs/srv/detail/calib_config_read__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<realsense2_camera_msgs::srv::CalibConfigRead_Request>()
{
  return "realsense2_camera_msgs::srv::CalibConfigRead_Request";
}

template<>
inline const char * name<realsense2_camera_msgs::srv::CalibConfigRead_Request>()
{
  return "realsense2_camera_msgs/srv/CalibConfigRead_Request";
}

template<>
struct has_fixed_size<realsense2_camera_msgs::srv::CalibConfigRead_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<realsense2_camera_msgs::srv::CalibConfigRead_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<realsense2_camera_msgs::srv::CalibConfigRead_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<realsense2_camera_msgs::srv::CalibConfigRead_Response>()
{
  return "realsense2_camera_msgs::srv::CalibConfigRead_Response";
}

template<>
inline const char * name<realsense2_camera_msgs::srv::CalibConfigRead_Response>()
{
  return "realsense2_camera_msgs/srv/CalibConfigRead_Response";
}

template<>
struct has_fixed_size<realsense2_camera_msgs::srv::CalibConfigRead_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<realsense2_camera_msgs::srv::CalibConfigRead_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<realsense2_camera_msgs::srv::CalibConfigRead_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<realsense2_camera_msgs::srv::CalibConfigRead>()
{
  return "realsense2_camera_msgs::srv::CalibConfigRead";
}

template<>
inline const char * name<realsense2_camera_msgs::srv::CalibConfigRead>()
{
  return "realsense2_camera_msgs/srv/CalibConfigRead";
}

template<>
struct has_fixed_size<realsense2_camera_msgs::srv::CalibConfigRead>
  : std::integral_constant<
    bool,
    has_fixed_size<realsense2_camera_msgs::srv::CalibConfigRead_Request>::value &&
    has_fixed_size<realsense2_camera_msgs::srv::CalibConfigRead_Response>::value
  >
{
};

template<>
struct has_bounded_size<realsense2_camera_msgs::srv::CalibConfigRead>
  : std::integral_constant<
    bool,
    has_bounded_size<realsense2_camera_msgs::srv::CalibConfigRead_Request>::value &&
    has_bounded_size<realsense2_camera_msgs::srv::CalibConfigRead_Response>::value
  >
{
};

template<>
struct is_service<realsense2_camera_msgs::srv::CalibConfigRead>
  : std::true_type
{
};

template<>
struct is_service_request<realsense2_camera_msgs::srv::CalibConfigRead_Request>
  : std::true_type
{
};

template<>
struct is_service_response<realsense2_camera_msgs::srv::CalibConfigRead_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // REALSENSE2_CAMERA_MSGS__SRV__DETAIL__CALIB_CONFIG_READ__TRAITS_HPP_
