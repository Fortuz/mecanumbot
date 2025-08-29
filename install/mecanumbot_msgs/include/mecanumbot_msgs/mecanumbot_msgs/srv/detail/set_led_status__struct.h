// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mecanumbot_msgs:srv/SetLedStatus.idl
// generated code does not contain a copyright notice

#ifndef MECANUMBOT_MSGS__SRV__DETAIL__SET_LED_STATUS__STRUCT_H_
#define MECANUMBOT_MSGS__SRV__DETAIL__SET_LED_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/SetLedStatus in the package mecanumbot_msgs.
typedef struct mecanumbot_msgs__srv__SetLedStatus_Request
{
  int8_t fl_mode;
  int8_t fl_color;
  int8_t fr_mode;
  int8_t fr_color;
  int8_t br_mode;
  int8_t br_color;
  int8_t bl_mode;
  int8_t bl_color;
} mecanumbot_msgs__srv__SetLedStatus_Request;

// Struct for a sequence of mecanumbot_msgs__srv__SetLedStatus_Request.
typedef struct mecanumbot_msgs__srv__SetLedStatus_Request__Sequence
{
  mecanumbot_msgs__srv__SetLedStatus_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mecanumbot_msgs__srv__SetLedStatus_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/SetLedStatus in the package mecanumbot_msgs.
typedef struct mecanumbot_msgs__srv__SetLedStatus_Response
{
  bool success;
  rosidl_runtime_c__String message;
} mecanumbot_msgs__srv__SetLedStatus_Response;

// Struct for a sequence of mecanumbot_msgs__srv__SetLedStatus_Response.
typedef struct mecanumbot_msgs__srv__SetLedStatus_Response__Sequence
{
  mecanumbot_msgs__srv__SetLedStatus_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mecanumbot_msgs__srv__SetLedStatus_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MECANUMBOT_MSGS__SRV__DETAIL__SET_LED_STATUS__STRUCT_H_
