// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mecanumbot_msgs:srv/SetLedStatus.idl
// generated code does not contain a copyright notice

#ifndef MECANUMBOT_MSGS__SRV__DETAIL__SET_LED_STATUS__STRUCT_HPP_
#define MECANUMBOT_MSGS__SRV__DETAIL__SET_LED_STATUS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__mecanumbot_msgs__srv__SetLedStatus_Request __attribute__((deprecated))
#else
# define DEPRECATED__mecanumbot_msgs__srv__SetLedStatus_Request __declspec(deprecated)
#endif

namespace mecanumbot_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetLedStatus_Request_
{
  using Type = SetLedStatus_Request_<ContainerAllocator>;

  explicit SetLedStatus_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->fl_mode = 0;
      this->fl_color = 0;
      this->fr_mode = 0;
      this->fr_color = 0;
      this->br_mode = 0;
      this->br_color = 0;
      this->bl_mode = 0;
      this->bl_color = 0;
    }
  }

  explicit SetLedStatus_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->fl_mode = 0;
      this->fl_color = 0;
      this->fr_mode = 0;
      this->fr_color = 0;
      this->br_mode = 0;
      this->br_color = 0;
      this->bl_mode = 0;
      this->bl_color = 0;
    }
  }

  // field types and members
  using _fl_mode_type =
    int8_t;
  _fl_mode_type fl_mode;
  using _fl_color_type =
    int8_t;
  _fl_color_type fl_color;
  using _fr_mode_type =
    int8_t;
  _fr_mode_type fr_mode;
  using _fr_color_type =
    int8_t;
  _fr_color_type fr_color;
  using _br_mode_type =
    int8_t;
  _br_mode_type br_mode;
  using _br_color_type =
    int8_t;
  _br_color_type br_color;
  using _bl_mode_type =
    int8_t;
  _bl_mode_type bl_mode;
  using _bl_color_type =
    int8_t;
  _bl_color_type bl_color;

  // setters for named parameter idiom
  Type & set__fl_mode(
    const int8_t & _arg)
  {
    this->fl_mode = _arg;
    return *this;
  }
  Type & set__fl_color(
    const int8_t & _arg)
  {
    this->fl_color = _arg;
    return *this;
  }
  Type & set__fr_mode(
    const int8_t & _arg)
  {
    this->fr_mode = _arg;
    return *this;
  }
  Type & set__fr_color(
    const int8_t & _arg)
  {
    this->fr_color = _arg;
    return *this;
  }
  Type & set__br_mode(
    const int8_t & _arg)
  {
    this->br_mode = _arg;
    return *this;
  }
  Type & set__br_color(
    const int8_t & _arg)
  {
    this->br_color = _arg;
    return *this;
  }
  Type & set__bl_mode(
    const int8_t & _arg)
  {
    this->bl_mode = _arg;
    return *this;
  }
  Type & set__bl_color(
    const int8_t & _arg)
  {
    this->bl_color = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mecanumbot_msgs::srv::SetLedStatus_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const mecanumbot_msgs::srv::SetLedStatus_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mecanumbot_msgs::srv::SetLedStatus_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mecanumbot_msgs::srv::SetLedStatus_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mecanumbot_msgs::srv::SetLedStatus_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mecanumbot_msgs::srv::SetLedStatus_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mecanumbot_msgs::srv::SetLedStatus_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mecanumbot_msgs::srv::SetLedStatus_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mecanumbot_msgs::srv::SetLedStatus_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mecanumbot_msgs::srv::SetLedStatus_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mecanumbot_msgs__srv__SetLedStatus_Request
    std::shared_ptr<mecanumbot_msgs::srv::SetLedStatus_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mecanumbot_msgs__srv__SetLedStatus_Request
    std::shared_ptr<mecanumbot_msgs::srv::SetLedStatus_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetLedStatus_Request_ & other) const
  {
    if (this->fl_mode != other.fl_mode) {
      return false;
    }
    if (this->fl_color != other.fl_color) {
      return false;
    }
    if (this->fr_mode != other.fr_mode) {
      return false;
    }
    if (this->fr_color != other.fr_color) {
      return false;
    }
    if (this->br_mode != other.br_mode) {
      return false;
    }
    if (this->br_color != other.br_color) {
      return false;
    }
    if (this->bl_mode != other.bl_mode) {
      return false;
    }
    if (this->bl_color != other.bl_color) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetLedStatus_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetLedStatus_Request_

// alias to use template instance with default allocator
using SetLedStatus_Request =
  mecanumbot_msgs::srv::SetLedStatus_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace mecanumbot_msgs


#ifndef _WIN32
# define DEPRECATED__mecanumbot_msgs__srv__SetLedStatus_Response __attribute__((deprecated))
#else
# define DEPRECATED__mecanumbot_msgs__srv__SetLedStatus_Response __declspec(deprecated)
#endif

namespace mecanumbot_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetLedStatus_Response_
{
  using Type = SetLedStatus_Response_<ContainerAllocator>;

  explicit SetLedStatus_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  explicit SetLedStatus_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : message(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _message_type message;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }
  Type & set__message(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->message = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mecanumbot_msgs::srv::SetLedStatus_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const mecanumbot_msgs::srv::SetLedStatus_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mecanumbot_msgs::srv::SetLedStatus_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mecanumbot_msgs::srv::SetLedStatus_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mecanumbot_msgs::srv::SetLedStatus_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mecanumbot_msgs::srv::SetLedStatus_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mecanumbot_msgs::srv::SetLedStatus_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mecanumbot_msgs::srv::SetLedStatus_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mecanumbot_msgs::srv::SetLedStatus_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mecanumbot_msgs::srv::SetLedStatus_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mecanumbot_msgs__srv__SetLedStatus_Response
    std::shared_ptr<mecanumbot_msgs::srv::SetLedStatus_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mecanumbot_msgs__srv__SetLedStatus_Response
    std::shared_ptr<mecanumbot_msgs::srv::SetLedStatus_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetLedStatus_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetLedStatus_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetLedStatus_Response_

// alias to use template instance with default allocator
using SetLedStatus_Response =
  mecanumbot_msgs::srv::SetLedStatus_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace mecanumbot_msgs

namespace mecanumbot_msgs
{

namespace srv
{

struct SetLedStatus
{
  using Request = mecanumbot_msgs::srv::SetLedStatus_Request;
  using Response = mecanumbot_msgs::srv::SetLedStatus_Response;
};

}  // namespace srv

}  // namespace mecanumbot_msgs

#endif  // MECANUMBOT_MSGS__SRV__DETAIL__SET_LED_STATUS__STRUCT_HPP_
