// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mecanumbot_msgs:msg/SensorState.idl
// generated code does not contain a copyright notice

#ifndef MECANUMBOT_MSGS__MSG__DETAIL__SENSOR_STATE__BUILDER_HPP_
#define MECANUMBOT_MSGS__MSG__DETAIL__SENSOR_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mecanumbot_msgs/msg/detail/sensor_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mecanumbot_msgs
{

namespace msg
{

namespace builder
{

class Init_SensorState_battery
{
public:
  explicit Init_SensorState_battery(::mecanumbot_msgs::msg::SensorState & msg)
  : msg_(msg)
  {}
  ::mecanumbot_msgs::msg::SensorState battery(::mecanumbot_msgs::msg::SensorState::_battery_type arg)
  {
    msg_.battery = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mecanumbot_msgs::msg::SensorState msg_;
};

class Init_SensorState_backleft_encoder
{
public:
  explicit Init_SensorState_backleft_encoder(::mecanumbot_msgs::msg::SensorState & msg)
  : msg_(msg)
  {}
  Init_SensorState_battery backleft_encoder(::mecanumbot_msgs::msg::SensorState::_backleft_encoder_type arg)
  {
    msg_.backleft_encoder = std::move(arg);
    return Init_SensorState_battery(msg_);
  }

private:
  ::mecanumbot_msgs::msg::SensorState msg_;
};

class Init_SensorState_backright_encoder
{
public:
  explicit Init_SensorState_backright_encoder(::mecanumbot_msgs::msg::SensorState & msg)
  : msg_(msg)
  {}
  Init_SensorState_backleft_encoder backright_encoder(::mecanumbot_msgs::msg::SensorState::_backright_encoder_type arg)
  {
    msg_.backright_encoder = std::move(arg);
    return Init_SensorState_backleft_encoder(msg_);
  }

private:
  ::mecanumbot_msgs::msg::SensorState msg_;
};

class Init_SensorState_frontright_encoder
{
public:
  explicit Init_SensorState_frontright_encoder(::mecanumbot_msgs::msg::SensorState & msg)
  : msg_(msg)
  {}
  Init_SensorState_backright_encoder frontright_encoder(::mecanumbot_msgs::msg::SensorState::_frontright_encoder_type arg)
  {
    msg_.frontright_encoder = std::move(arg);
    return Init_SensorState_backright_encoder(msg_);
  }

private:
  ::mecanumbot_msgs::msg::SensorState msg_;
};

class Init_SensorState_frontleft_encoder
{
public:
  explicit Init_SensorState_frontleft_encoder(::mecanumbot_msgs::msg::SensorState & msg)
  : msg_(msg)
  {}
  Init_SensorState_frontright_encoder frontleft_encoder(::mecanumbot_msgs::msg::SensorState::_frontleft_encoder_type arg)
  {
    msg_.frontleft_encoder = std::move(arg);
    return Init_SensorState_frontright_encoder(msg_);
  }

private:
  ::mecanumbot_msgs::msg::SensorState msg_;
};

class Init_SensorState_torque
{
public:
  explicit Init_SensorState_torque(::mecanumbot_msgs::msg::SensorState & msg)
  : msg_(msg)
  {}
  Init_SensorState_frontleft_encoder torque(::mecanumbot_msgs::msg::SensorState::_torque_type arg)
  {
    msg_.torque = std::move(arg);
    return Init_SensorState_frontleft_encoder(msg_);
  }

private:
  ::mecanumbot_msgs::msg::SensorState msg_;
};

class Init_SensorState_button
{
public:
  explicit Init_SensorState_button(::mecanumbot_msgs::msg::SensorState & msg)
  : msg_(msg)
  {}
  Init_SensorState_torque button(::mecanumbot_msgs::msg::SensorState::_button_type arg)
  {
    msg_.button = std::move(arg);
    return Init_SensorState_torque(msg_);
  }

private:
  ::mecanumbot_msgs::msg::SensorState msg_;
};

class Init_SensorState_led
{
public:
  explicit Init_SensorState_led(::mecanumbot_msgs::msg::SensorState & msg)
  : msg_(msg)
  {}
  Init_SensorState_button led(::mecanumbot_msgs::msg::SensorState::_led_type arg)
  {
    msg_.led = std::move(arg);
    return Init_SensorState_button(msg_);
  }

private:
  ::mecanumbot_msgs::msg::SensorState msg_;
};

class Init_SensorState_illumination
{
public:
  explicit Init_SensorState_illumination(::mecanumbot_msgs::msg::SensorState & msg)
  : msg_(msg)
  {}
  Init_SensorState_led illumination(::mecanumbot_msgs::msg::SensorState::_illumination_type arg)
  {
    msg_.illumination = std::move(arg);
    return Init_SensorState_led(msg_);
  }

private:
  ::mecanumbot_msgs::msg::SensorState msg_;
};

class Init_SensorState_sonar
{
public:
  explicit Init_SensorState_sonar(::mecanumbot_msgs::msg::SensorState & msg)
  : msg_(msg)
  {}
  Init_SensorState_illumination sonar(::mecanumbot_msgs::msg::SensorState::_sonar_type arg)
  {
    msg_.sonar = std::move(arg);
    return Init_SensorState_illumination(msg_);
  }

private:
  ::mecanumbot_msgs::msg::SensorState msg_;
};

class Init_SensorState_cliff
{
public:
  explicit Init_SensorState_cliff(::mecanumbot_msgs::msg::SensorState & msg)
  : msg_(msg)
  {}
  Init_SensorState_sonar cliff(::mecanumbot_msgs::msg::SensorState::_cliff_type arg)
  {
    msg_.cliff = std::move(arg);
    return Init_SensorState_sonar(msg_);
  }

private:
  ::mecanumbot_msgs::msg::SensorState msg_;
};

class Init_SensorState_bumper
{
public:
  explicit Init_SensorState_bumper(::mecanumbot_msgs::msg::SensorState & msg)
  : msg_(msg)
  {}
  Init_SensorState_cliff bumper(::mecanumbot_msgs::msg::SensorState::_bumper_type arg)
  {
    msg_.bumper = std::move(arg);
    return Init_SensorState_cliff(msg_);
  }

private:
  ::mecanumbot_msgs::msg::SensorState msg_;
};

class Init_SensorState_header
{
public:
  Init_SensorState_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SensorState_bumper header(::mecanumbot_msgs::msg::SensorState::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_SensorState_bumper(msg_);
  }

private:
  ::mecanumbot_msgs::msg::SensorState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mecanumbot_msgs::msg::SensorState>()
{
  return mecanumbot_msgs::msg::builder::Init_SensorState_header();
}

}  // namespace mecanumbot_msgs

#endif  // MECANUMBOT_MSGS__MSG__DETAIL__SENSOR_STATE__BUILDER_HPP_
