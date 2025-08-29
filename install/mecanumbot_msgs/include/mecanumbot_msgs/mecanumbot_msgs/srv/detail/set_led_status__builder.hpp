// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mecanumbot_msgs:srv/SetLedStatus.idl
// generated code does not contain a copyright notice

#ifndef MECANUMBOT_MSGS__SRV__DETAIL__SET_LED_STATUS__BUILDER_HPP_
#define MECANUMBOT_MSGS__SRV__DETAIL__SET_LED_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mecanumbot_msgs/srv/detail/set_led_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mecanumbot_msgs
{

namespace srv
{

namespace builder
{

class Init_SetLedStatus_Request_bl_color
{
public:
  explicit Init_SetLedStatus_Request_bl_color(::mecanumbot_msgs::srv::SetLedStatus_Request & msg)
  : msg_(msg)
  {}
  ::mecanumbot_msgs::srv::SetLedStatus_Request bl_color(::mecanumbot_msgs::srv::SetLedStatus_Request::_bl_color_type arg)
  {
    msg_.bl_color = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mecanumbot_msgs::srv::SetLedStatus_Request msg_;
};

class Init_SetLedStatus_Request_bl_mode
{
public:
  explicit Init_SetLedStatus_Request_bl_mode(::mecanumbot_msgs::srv::SetLedStatus_Request & msg)
  : msg_(msg)
  {}
  Init_SetLedStatus_Request_bl_color bl_mode(::mecanumbot_msgs::srv::SetLedStatus_Request::_bl_mode_type arg)
  {
    msg_.bl_mode = std::move(arg);
    return Init_SetLedStatus_Request_bl_color(msg_);
  }

private:
  ::mecanumbot_msgs::srv::SetLedStatus_Request msg_;
};

class Init_SetLedStatus_Request_br_color
{
public:
  explicit Init_SetLedStatus_Request_br_color(::mecanumbot_msgs::srv::SetLedStatus_Request & msg)
  : msg_(msg)
  {}
  Init_SetLedStatus_Request_bl_mode br_color(::mecanumbot_msgs::srv::SetLedStatus_Request::_br_color_type arg)
  {
    msg_.br_color = std::move(arg);
    return Init_SetLedStatus_Request_bl_mode(msg_);
  }

private:
  ::mecanumbot_msgs::srv::SetLedStatus_Request msg_;
};

class Init_SetLedStatus_Request_br_mode
{
public:
  explicit Init_SetLedStatus_Request_br_mode(::mecanumbot_msgs::srv::SetLedStatus_Request & msg)
  : msg_(msg)
  {}
  Init_SetLedStatus_Request_br_color br_mode(::mecanumbot_msgs::srv::SetLedStatus_Request::_br_mode_type arg)
  {
    msg_.br_mode = std::move(arg);
    return Init_SetLedStatus_Request_br_color(msg_);
  }

private:
  ::mecanumbot_msgs::srv::SetLedStatus_Request msg_;
};

class Init_SetLedStatus_Request_fr_color
{
public:
  explicit Init_SetLedStatus_Request_fr_color(::mecanumbot_msgs::srv::SetLedStatus_Request & msg)
  : msg_(msg)
  {}
  Init_SetLedStatus_Request_br_mode fr_color(::mecanumbot_msgs::srv::SetLedStatus_Request::_fr_color_type arg)
  {
    msg_.fr_color = std::move(arg);
    return Init_SetLedStatus_Request_br_mode(msg_);
  }

private:
  ::mecanumbot_msgs::srv::SetLedStatus_Request msg_;
};

class Init_SetLedStatus_Request_fr_mode
{
public:
  explicit Init_SetLedStatus_Request_fr_mode(::mecanumbot_msgs::srv::SetLedStatus_Request & msg)
  : msg_(msg)
  {}
  Init_SetLedStatus_Request_fr_color fr_mode(::mecanumbot_msgs::srv::SetLedStatus_Request::_fr_mode_type arg)
  {
    msg_.fr_mode = std::move(arg);
    return Init_SetLedStatus_Request_fr_color(msg_);
  }

private:
  ::mecanumbot_msgs::srv::SetLedStatus_Request msg_;
};

class Init_SetLedStatus_Request_fl_color
{
public:
  explicit Init_SetLedStatus_Request_fl_color(::mecanumbot_msgs::srv::SetLedStatus_Request & msg)
  : msg_(msg)
  {}
  Init_SetLedStatus_Request_fr_mode fl_color(::mecanumbot_msgs::srv::SetLedStatus_Request::_fl_color_type arg)
  {
    msg_.fl_color = std::move(arg);
    return Init_SetLedStatus_Request_fr_mode(msg_);
  }

private:
  ::mecanumbot_msgs::srv::SetLedStatus_Request msg_;
};

class Init_SetLedStatus_Request_fl_mode
{
public:
  Init_SetLedStatus_Request_fl_mode()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetLedStatus_Request_fl_color fl_mode(::mecanumbot_msgs::srv::SetLedStatus_Request::_fl_mode_type arg)
  {
    msg_.fl_mode = std::move(arg);
    return Init_SetLedStatus_Request_fl_color(msg_);
  }

private:
  ::mecanumbot_msgs::srv::SetLedStatus_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mecanumbot_msgs::srv::SetLedStatus_Request>()
{
  return mecanumbot_msgs::srv::builder::Init_SetLedStatus_Request_fl_mode();
}

}  // namespace mecanumbot_msgs


namespace mecanumbot_msgs
{

namespace srv
{

namespace builder
{

class Init_SetLedStatus_Response_message
{
public:
  explicit Init_SetLedStatus_Response_message(::mecanumbot_msgs::srv::SetLedStatus_Response & msg)
  : msg_(msg)
  {}
  ::mecanumbot_msgs::srv::SetLedStatus_Response message(::mecanumbot_msgs::srv::SetLedStatus_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mecanumbot_msgs::srv::SetLedStatus_Response msg_;
};

class Init_SetLedStatus_Response_success
{
public:
  Init_SetLedStatus_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetLedStatus_Response_message success(::mecanumbot_msgs::srv::SetLedStatus_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_SetLedStatus_Response_message(msg_);
  }

private:
  ::mecanumbot_msgs::srv::SetLedStatus_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mecanumbot_msgs::srv::SetLedStatus_Response>()
{
  return mecanumbot_msgs::srv::builder::Init_SetLedStatus_Response_success();
}

}  // namespace mecanumbot_msgs

#endif  // MECANUMBOT_MSGS__SRV__DETAIL__SET_LED_STATUS__BUILDER_HPP_
