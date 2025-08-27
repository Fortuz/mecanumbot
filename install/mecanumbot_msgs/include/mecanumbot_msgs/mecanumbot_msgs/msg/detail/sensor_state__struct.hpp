// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mecanumbot_msgs:msg/SensorState.idl
// generated code does not contain a copyright notice

#ifndef MECANUMBOT_MSGS__MSG__DETAIL__SENSOR_STATE__STRUCT_HPP_
#define MECANUMBOT_MSGS__MSG__DETAIL__SENSOR_STATE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__mecanumbot_msgs__msg__SensorState __attribute__((deprecated))
#else
# define DEPRECATED__mecanumbot_msgs__msg__SensorState __declspec(deprecated)
#endif

namespace mecanumbot_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct SensorState_
{
  using Type = SensorState_<ContainerAllocator>;

  explicit SensorState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->bumper = 0;
      this->cliff = 0.0f;
      this->sonar = 0.0f;
      this->illumination = 0.0f;
      this->led = 0;
      this->button = 0;
      this->torque = false;
      this->frontleft_encoder = 0l;
      this->frontright_encoder = 0l;
      this->backright_encoder = 0l;
      this->backleft_encoder = 0l;
      this->battery = 0.0f;
    }
  }

  explicit SensorState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->bumper = 0;
      this->cliff = 0.0f;
      this->sonar = 0.0f;
      this->illumination = 0.0f;
      this->led = 0;
      this->button = 0;
      this->torque = false;
      this->frontleft_encoder = 0l;
      this->frontright_encoder = 0l;
      this->backright_encoder = 0l;
      this->backleft_encoder = 0l;
      this->battery = 0.0f;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _bumper_type =
    uint8_t;
  _bumper_type bumper;
  using _cliff_type =
    float;
  _cliff_type cliff;
  using _sonar_type =
    float;
  _sonar_type sonar;
  using _illumination_type =
    float;
  _illumination_type illumination;
  using _led_type =
    uint8_t;
  _led_type led;
  using _button_type =
    uint8_t;
  _button_type button;
  using _torque_type =
    bool;
  _torque_type torque;
  using _frontleft_encoder_type =
    int32_t;
  _frontleft_encoder_type frontleft_encoder;
  using _frontright_encoder_type =
    int32_t;
  _frontright_encoder_type frontright_encoder;
  using _backright_encoder_type =
    int32_t;
  _backright_encoder_type backright_encoder;
  using _backleft_encoder_type =
    int32_t;
  _backleft_encoder_type backleft_encoder;
  using _battery_type =
    float;
  _battery_type battery;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__bumper(
    const uint8_t & _arg)
  {
    this->bumper = _arg;
    return *this;
  }
  Type & set__cliff(
    const float & _arg)
  {
    this->cliff = _arg;
    return *this;
  }
  Type & set__sonar(
    const float & _arg)
  {
    this->sonar = _arg;
    return *this;
  }
  Type & set__illumination(
    const float & _arg)
  {
    this->illumination = _arg;
    return *this;
  }
  Type & set__led(
    const uint8_t & _arg)
  {
    this->led = _arg;
    return *this;
  }
  Type & set__button(
    const uint8_t & _arg)
  {
    this->button = _arg;
    return *this;
  }
  Type & set__torque(
    const bool & _arg)
  {
    this->torque = _arg;
    return *this;
  }
  Type & set__frontleft_encoder(
    const int32_t & _arg)
  {
    this->frontleft_encoder = _arg;
    return *this;
  }
  Type & set__frontright_encoder(
    const int32_t & _arg)
  {
    this->frontright_encoder = _arg;
    return *this;
  }
  Type & set__backright_encoder(
    const int32_t & _arg)
  {
    this->backright_encoder = _arg;
    return *this;
  }
  Type & set__backleft_encoder(
    const int32_t & _arg)
  {
    this->backleft_encoder = _arg;
    return *this;
  }
  Type & set__battery(
    const float & _arg)
  {
    this->battery = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t BUMPER_FORWARD =
    1u;
  static constexpr uint8_t BUMPER_BACKWARD =
    2u;
  static constexpr uint8_t CLIFF =
    1u;
  static constexpr uint8_t SONAR =
    1u;
  static constexpr uint8_t ILLUMINATION =
    1u;
  static constexpr uint8_t BUTTON0 =
    1u;
  static constexpr uint8_t BUTTON1 =
    2u;
  static constexpr uint8_t ERROR_FRONTLEFT_MOTOR =
    1u;
  static constexpr uint8_t ERROR_FRONTRIGHT_MOTOR =
    2u;
  static constexpr uint8_t ERROR_BACKRIGHT_MOTOR =
    3u;
  static constexpr uint8_t ERROR_BACKLEFT_MOTOR =
    4u;
  static constexpr uint8_t ERROR_NECK_MOTOR =
    5u;
  static constexpr uint8_t ERROR_GRABBERLEFT_MOTOR =
    6u;
  static constexpr uint8_t ERROR_GRABBERRIGHT_MOTOR =
    7u;
  static constexpr uint8_t TORQUE_ON =
    1u;
  static constexpr uint8_t TORQUE_OFF =
    2u;

  // pointer types
  using RawPtr =
    mecanumbot_msgs::msg::SensorState_<ContainerAllocator> *;
  using ConstRawPtr =
    const mecanumbot_msgs::msg::SensorState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mecanumbot_msgs::msg::SensorState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mecanumbot_msgs::msg::SensorState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mecanumbot_msgs::msg::SensorState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mecanumbot_msgs::msg::SensorState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mecanumbot_msgs::msg::SensorState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mecanumbot_msgs::msg::SensorState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mecanumbot_msgs::msg::SensorState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mecanumbot_msgs::msg::SensorState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mecanumbot_msgs__msg__SensorState
    std::shared_ptr<mecanumbot_msgs::msg::SensorState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mecanumbot_msgs__msg__SensorState
    std::shared_ptr<mecanumbot_msgs::msg::SensorState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SensorState_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->bumper != other.bumper) {
      return false;
    }
    if (this->cliff != other.cliff) {
      return false;
    }
    if (this->sonar != other.sonar) {
      return false;
    }
    if (this->illumination != other.illumination) {
      return false;
    }
    if (this->led != other.led) {
      return false;
    }
    if (this->button != other.button) {
      return false;
    }
    if (this->torque != other.torque) {
      return false;
    }
    if (this->frontleft_encoder != other.frontleft_encoder) {
      return false;
    }
    if (this->frontright_encoder != other.frontright_encoder) {
      return false;
    }
    if (this->backright_encoder != other.backright_encoder) {
      return false;
    }
    if (this->backleft_encoder != other.backleft_encoder) {
      return false;
    }
    if (this->battery != other.battery) {
      return false;
    }
    return true;
  }
  bool operator!=(const SensorState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SensorState_

// alias to use template instance with default allocator
using SensorState =
  mecanumbot_msgs::msg::SensorState_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SensorState_<ContainerAllocator>::BUMPER_FORWARD;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SensorState_<ContainerAllocator>::BUMPER_BACKWARD;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SensorState_<ContainerAllocator>::CLIFF;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SensorState_<ContainerAllocator>::SONAR;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SensorState_<ContainerAllocator>::ILLUMINATION;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SensorState_<ContainerAllocator>::BUTTON0;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SensorState_<ContainerAllocator>::BUTTON1;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SensorState_<ContainerAllocator>::ERROR_FRONTLEFT_MOTOR;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SensorState_<ContainerAllocator>::ERROR_FRONTRIGHT_MOTOR;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SensorState_<ContainerAllocator>::ERROR_BACKRIGHT_MOTOR;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SensorState_<ContainerAllocator>::ERROR_BACKLEFT_MOTOR;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SensorState_<ContainerAllocator>::ERROR_NECK_MOTOR;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SensorState_<ContainerAllocator>::ERROR_GRABBERLEFT_MOTOR;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SensorState_<ContainerAllocator>::ERROR_GRABBERRIGHT_MOTOR;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SensorState_<ContainerAllocator>::TORQUE_ON;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SensorState_<ContainerAllocator>::TORQUE_OFF;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace mecanumbot_msgs

#endif  // MECANUMBOT_MSGS__MSG__DETAIL__SENSOR_STATE__STRUCT_HPP_
