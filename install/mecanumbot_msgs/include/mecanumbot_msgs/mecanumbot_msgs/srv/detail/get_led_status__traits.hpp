// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from mecanumbot_msgs:srv/GetLedStatus.idl
// generated code does not contain a copyright notice

#ifndef MECANUMBOT_MSGS__SRV__DETAIL__GET_LED_STATUS__TRAITS_HPP_
#define MECANUMBOT_MSGS__SRV__DETAIL__GET_LED_STATUS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "mecanumbot_msgs/srv/detail/get_led_status__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace mecanumbot_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const GetLedStatus_Request & msg,
  std::ostream & out)
{
  (void)msg;
  out << "null";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GetLedStatus_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  (void)msg;
  (void)indentation;
  out << "null\n";
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GetLedStatus_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace mecanumbot_msgs

namespace rosidl_generator_traits
{

[[deprecated("use mecanumbot_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const mecanumbot_msgs::srv::GetLedStatus_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  mecanumbot_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mecanumbot_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const mecanumbot_msgs::srv::GetLedStatus_Request & msg)
{
  return mecanumbot_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<mecanumbot_msgs::srv::GetLedStatus_Request>()
{
  return "mecanumbot_msgs::srv::GetLedStatus_Request";
}

template<>
inline const char * name<mecanumbot_msgs::srv::GetLedStatus_Request>()
{
  return "mecanumbot_msgs/srv/GetLedStatus_Request";
}

template<>
struct has_fixed_size<mecanumbot_msgs::srv::GetLedStatus_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<mecanumbot_msgs::srv::GetLedStatus_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<mecanumbot_msgs::srv::GetLedStatus_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace mecanumbot_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const GetLedStatus_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: fl_mode
  {
    out << "fl_mode: ";
    rosidl_generator_traits::value_to_yaml(msg.fl_mode, out);
    out << ", ";
  }

  // member: fl_color
  {
    out << "fl_color: ";
    rosidl_generator_traits::value_to_yaml(msg.fl_color, out);
    out << ", ";
  }

  // member: fr_mode
  {
    out << "fr_mode: ";
    rosidl_generator_traits::value_to_yaml(msg.fr_mode, out);
    out << ", ";
  }

  // member: fr_color
  {
    out << "fr_color: ";
    rosidl_generator_traits::value_to_yaml(msg.fr_color, out);
    out << ", ";
  }

  // member: br_mode
  {
    out << "br_mode: ";
    rosidl_generator_traits::value_to_yaml(msg.br_mode, out);
    out << ", ";
  }

  // member: br_color
  {
    out << "br_color: ";
    rosidl_generator_traits::value_to_yaml(msg.br_color, out);
    out << ", ";
  }

  // member: bl_mode
  {
    out << "bl_mode: ";
    rosidl_generator_traits::value_to_yaml(msg.bl_mode, out);
    out << ", ";
  }

  // member: bl_color
  {
    out << "bl_color: ";
    rosidl_generator_traits::value_to_yaml(msg.bl_color, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GetLedStatus_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: fl_mode
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "fl_mode: ";
    rosidl_generator_traits::value_to_yaml(msg.fl_mode, out);
    out << "\n";
  }

  // member: fl_color
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "fl_color: ";
    rosidl_generator_traits::value_to_yaml(msg.fl_color, out);
    out << "\n";
  }

  // member: fr_mode
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "fr_mode: ";
    rosidl_generator_traits::value_to_yaml(msg.fr_mode, out);
    out << "\n";
  }

  // member: fr_color
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "fr_color: ";
    rosidl_generator_traits::value_to_yaml(msg.fr_color, out);
    out << "\n";
  }

  // member: br_mode
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "br_mode: ";
    rosidl_generator_traits::value_to_yaml(msg.br_mode, out);
    out << "\n";
  }

  // member: br_color
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "br_color: ";
    rosidl_generator_traits::value_to_yaml(msg.br_color, out);
    out << "\n";
  }

  // member: bl_mode
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "bl_mode: ";
    rosidl_generator_traits::value_to_yaml(msg.bl_mode, out);
    out << "\n";
  }

  // member: bl_color
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "bl_color: ";
    rosidl_generator_traits::value_to_yaml(msg.bl_color, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GetLedStatus_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace mecanumbot_msgs

namespace rosidl_generator_traits
{

[[deprecated("use mecanumbot_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const mecanumbot_msgs::srv::GetLedStatus_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  mecanumbot_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mecanumbot_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const mecanumbot_msgs::srv::GetLedStatus_Response & msg)
{
  return mecanumbot_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<mecanumbot_msgs::srv::GetLedStatus_Response>()
{
  return "mecanumbot_msgs::srv::GetLedStatus_Response";
}

template<>
inline const char * name<mecanumbot_msgs::srv::GetLedStatus_Response>()
{
  return "mecanumbot_msgs/srv/GetLedStatus_Response";
}

template<>
struct has_fixed_size<mecanumbot_msgs::srv::GetLedStatus_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<mecanumbot_msgs::srv::GetLedStatus_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<mecanumbot_msgs::srv::GetLedStatus_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<mecanumbot_msgs::srv::GetLedStatus>()
{
  return "mecanumbot_msgs::srv::GetLedStatus";
}

template<>
inline const char * name<mecanumbot_msgs::srv::GetLedStatus>()
{
  return "mecanumbot_msgs/srv/GetLedStatus";
}

template<>
struct has_fixed_size<mecanumbot_msgs::srv::GetLedStatus>
  : std::integral_constant<
    bool,
    has_fixed_size<mecanumbot_msgs::srv::GetLedStatus_Request>::value &&
    has_fixed_size<mecanumbot_msgs::srv::GetLedStatus_Response>::value
  >
{
};

template<>
struct has_bounded_size<mecanumbot_msgs::srv::GetLedStatus>
  : std::integral_constant<
    bool,
    has_bounded_size<mecanumbot_msgs::srv::GetLedStatus_Request>::value &&
    has_bounded_size<mecanumbot_msgs::srv::GetLedStatus_Response>::value
  >
{
};

template<>
struct is_service<mecanumbot_msgs::srv::GetLedStatus>
  : std::true_type
{
};

template<>
struct is_service_request<mecanumbot_msgs::srv::GetLedStatus_Request>
  : std::true_type
{
};

template<>
struct is_service_response<mecanumbot_msgs::srv::GetLedStatus_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // MECANUMBOT_MSGS__SRV__DETAIL__GET_LED_STATUS__TRAITS_HPP_
