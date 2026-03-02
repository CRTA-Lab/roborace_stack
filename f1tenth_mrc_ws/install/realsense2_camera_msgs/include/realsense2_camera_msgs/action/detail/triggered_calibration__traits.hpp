// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from realsense2_camera_msgs:action/TriggeredCalibration.idl
// generated code does not contain a copyright notice

#ifndef REALSENSE2_CAMERA_MSGS__ACTION__DETAIL__TRIGGERED_CALIBRATION__TRAITS_HPP_
#define REALSENSE2_CAMERA_MSGS__ACTION__DETAIL__TRIGGERED_CALIBRATION__TRAITS_HPP_

#include "realsense2_camera_msgs/action/detail/triggered_calibration__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<realsense2_camera_msgs::action::TriggeredCalibration_Goal>()
{
  return "realsense2_camera_msgs::action::TriggeredCalibration_Goal";
}

template<>
inline const char * name<realsense2_camera_msgs::action::TriggeredCalibration_Goal>()
{
  return "realsense2_camera_msgs/action/TriggeredCalibration_Goal";
}

template<>
struct has_fixed_size<realsense2_camera_msgs::action::TriggeredCalibration_Goal>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<realsense2_camera_msgs::action::TriggeredCalibration_Goal>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<realsense2_camera_msgs::action::TriggeredCalibration_Goal>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<realsense2_camera_msgs::action::TriggeredCalibration_Result>()
{
  return "realsense2_camera_msgs::action::TriggeredCalibration_Result";
}

template<>
inline const char * name<realsense2_camera_msgs::action::TriggeredCalibration_Result>()
{
  return "realsense2_camera_msgs/action/TriggeredCalibration_Result";
}

template<>
struct has_fixed_size<realsense2_camera_msgs::action::TriggeredCalibration_Result>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<realsense2_camera_msgs::action::TriggeredCalibration_Result>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<realsense2_camera_msgs::action::TriggeredCalibration_Result>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<realsense2_camera_msgs::action::TriggeredCalibration_Feedback>()
{
  return "realsense2_camera_msgs::action::TriggeredCalibration_Feedback";
}

template<>
inline const char * name<realsense2_camera_msgs::action::TriggeredCalibration_Feedback>()
{
  return "realsense2_camera_msgs/action/TriggeredCalibration_Feedback";
}

template<>
struct has_fixed_size<realsense2_camera_msgs::action::TriggeredCalibration_Feedback>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<realsense2_camera_msgs::action::TriggeredCalibration_Feedback>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<realsense2_camera_msgs::action::TriggeredCalibration_Feedback>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"
// Member 'goal'
#include "realsense2_camera_msgs/action/detail/triggered_calibration__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<realsense2_camera_msgs::action::TriggeredCalibration_SendGoal_Request>()
{
  return "realsense2_camera_msgs::action::TriggeredCalibration_SendGoal_Request";
}

template<>
inline const char * name<realsense2_camera_msgs::action::TriggeredCalibration_SendGoal_Request>()
{
  return "realsense2_camera_msgs/action/TriggeredCalibration_SendGoal_Request";
}

template<>
struct has_fixed_size<realsense2_camera_msgs::action::TriggeredCalibration_SendGoal_Request>
  : std::integral_constant<bool, has_fixed_size<realsense2_camera_msgs::action::TriggeredCalibration_Goal>::value && has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<realsense2_camera_msgs::action::TriggeredCalibration_SendGoal_Request>
  : std::integral_constant<bool, has_bounded_size<realsense2_camera_msgs::action::TriggeredCalibration_Goal>::value && has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<realsense2_camera_msgs::action::TriggeredCalibration_SendGoal_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<realsense2_camera_msgs::action::TriggeredCalibration_SendGoal_Response>()
{
  return "realsense2_camera_msgs::action::TriggeredCalibration_SendGoal_Response";
}

template<>
inline const char * name<realsense2_camera_msgs::action::TriggeredCalibration_SendGoal_Response>()
{
  return "realsense2_camera_msgs/action/TriggeredCalibration_SendGoal_Response";
}

template<>
struct has_fixed_size<realsense2_camera_msgs::action::TriggeredCalibration_SendGoal_Response>
  : std::integral_constant<bool, has_fixed_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct has_bounded_size<realsense2_camera_msgs::action::TriggeredCalibration_SendGoal_Response>
  : std::integral_constant<bool, has_bounded_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct is_message<realsense2_camera_msgs::action::TriggeredCalibration_SendGoal_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<realsense2_camera_msgs::action::TriggeredCalibration_SendGoal>()
{
  return "realsense2_camera_msgs::action::TriggeredCalibration_SendGoal";
}

template<>
inline const char * name<realsense2_camera_msgs::action::TriggeredCalibration_SendGoal>()
{
  return "realsense2_camera_msgs/action/TriggeredCalibration_SendGoal";
}

template<>
struct has_fixed_size<realsense2_camera_msgs::action::TriggeredCalibration_SendGoal>
  : std::integral_constant<
    bool,
    has_fixed_size<realsense2_camera_msgs::action::TriggeredCalibration_SendGoal_Request>::value &&
    has_fixed_size<realsense2_camera_msgs::action::TriggeredCalibration_SendGoal_Response>::value
  >
{
};

template<>
struct has_bounded_size<realsense2_camera_msgs::action::TriggeredCalibration_SendGoal>
  : std::integral_constant<
    bool,
    has_bounded_size<realsense2_camera_msgs::action::TriggeredCalibration_SendGoal_Request>::value &&
    has_bounded_size<realsense2_camera_msgs::action::TriggeredCalibration_SendGoal_Response>::value
  >
{
};

template<>
struct is_service<realsense2_camera_msgs::action::TriggeredCalibration_SendGoal>
  : std::true_type
{
};

template<>
struct is_service_request<realsense2_camera_msgs::action::TriggeredCalibration_SendGoal_Request>
  : std::true_type
{
};

template<>
struct is_service_response<realsense2_camera_msgs::action::TriggeredCalibration_SendGoal_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<realsense2_camera_msgs::action::TriggeredCalibration_GetResult_Request>()
{
  return "realsense2_camera_msgs::action::TriggeredCalibration_GetResult_Request";
}

template<>
inline const char * name<realsense2_camera_msgs::action::TriggeredCalibration_GetResult_Request>()
{
  return "realsense2_camera_msgs/action/TriggeredCalibration_GetResult_Request";
}

template<>
struct has_fixed_size<realsense2_camera_msgs::action::TriggeredCalibration_GetResult_Request>
  : std::integral_constant<bool, has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<realsense2_camera_msgs::action::TriggeredCalibration_GetResult_Request>
  : std::integral_constant<bool, has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<realsense2_camera_msgs::action::TriggeredCalibration_GetResult_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'result'
// already included above
// #include "realsense2_camera_msgs/action/detail/triggered_calibration__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<realsense2_camera_msgs::action::TriggeredCalibration_GetResult_Response>()
{
  return "realsense2_camera_msgs::action::TriggeredCalibration_GetResult_Response";
}

template<>
inline const char * name<realsense2_camera_msgs::action::TriggeredCalibration_GetResult_Response>()
{
  return "realsense2_camera_msgs/action/TriggeredCalibration_GetResult_Response";
}

template<>
struct has_fixed_size<realsense2_camera_msgs::action::TriggeredCalibration_GetResult_Response>
  : std::integral_constant<bool, has_fixed_size<realsense2_camera_msgs::action::TriggeredCalibration_Result>::value> {};

template<>
struct has_bounded_size<realsense2_camera_msgs::action::TriggeredCalibration_GetResult_Response>
  : std::integral_constant<bool, has_bounded_size<realsense2_camera_msgs::action::TriggeredCalibration_Result>::value> {};

template<>
struct is_message<realsense2_camera_msgs::action::TriggeredCalibration_GetResult_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<realsense2_camera_msgs::action::TriggeredCalibration_GetResult>()
{
  return "realsense2_camera_msgs::action::TriggeredCalibration_GetResult";
}

template<>
inline const char * name<realsense2_camera_msgs::action::TriggeredCalibration_GetResult>()
{
  return "realsense2_camera_msgs/action/TriggeredCalibration_GetResult";
}

template<>
struct has_fixed_size<realsense2_camera_msgs::action::TriggeredCalibration_GetResult>
  : std::integral_constant<
    bool,
    has_fixed_size<realsense2_camera_msgs::action::TriggeredCalibration_GetResult_Request>::value &&
    has_fixed_size<realsense2_camera_msgs::action::TriggeredCalibration_GetResult_Response>::value
  >
{
};

template<>
struct has_bounded_size<realsense2_camera_msgs::action::TriggeredCalibration_GetResult>
  : std::integral_constant<
    bool,
    has_bounded_size<realsense2_camera_msgs::action::TriggeredCalibration_GetResult_Request>::value &&
    has_bounded_size<realsense2_camera_msgs::action::TriggeredCalibration_GetResult_Response>::value
  >
{
};

template<>
struct is_service<realsense2_camera_msgs::action::TriggeredCalibration_GetResult>
  : std::true_type
{
};

template<>
struct is_service_request<realsense2_camera_msgs::action::TriggeredCalibration_GetResult_Request>
  : std::true_type
{
};

template<>
struct is_service_response<realsense2_camera_msgs::action::TriggeredCalibration_GetResult_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"
// Member 'feedback'
// already included above
// #include "realsense2_camera_msgs/action/detail/triggered_calibration__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<realsense2_camera_msgs::action::TriggeredCalibration_FeedbackMessage>()
{
  return "realsense2_camera_msgs::action::TriggeredCalibration_FeedbackMessage";
}

template<>
inline const char * name<realsense2_camera_msgs::action::TriggeredCalibration_FeedbackMessage>()
{
  return "realsense2_camera_msgs/action/TriggeredCalibration_FeedbackMessage";
}

template<>
struct has_fixed_size<realsense2_camera_msgs::action::TriggeredCalibration_FeedbackMessage>
  : std::integral_constant<bool, has_fixed_size<realsense2_camera_msgs::action::TriggeredCalibration_Feedback>::value && has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<realsense2_camera_msgs::action::TriggeredCalibration_FeedbackMessage>
  : std::integral_constant<bool, has_bounded_size<realsense2_camera_msgs::action::TriggeredCalibration_Feedback>::value && has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<realsense2_camera_msgs::action::TriggeredCalibration_FeedbackMessage>
  : std::true_type {};

}  // namespace rosidl_generator_traits


namespace rosidl_generator_traits
{

template<>
struct is_action<realsense2_camera_msgs::action::TriggeredCalibration>
  : std::true_type
{
};

template<>
struct is_action_goal<realsense2_camera_msgs::action::TriggeredCalibration_Goal>
  : std::true_type
{
};

template<>
struct is_action_result<realsense2_camera_msgs::action::TriggeredCalibration_Result>
  : std::true_type
{
};

template<>
struct is_action_feedback<realsense2_camera_msgs::action::TriggeredCalibration_Feedback>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits


#endif  // REALSENSE2_CAMERA_MSGS__ACTION__DETAIL__TRIGGERED_CALIBRATION__TRAITS_HPP_
