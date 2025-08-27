// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from mecanumbot_msgs:srv/GetLedStatus.idl
// generated code does not contain a copyright notice

#ifndef MECANUMBOT_MSGS__SRV__DETAIL__GET_LED_STATUS__FUNCTIONS_H_
#define MECANUMBOT_MSGS__SRV__DETAIL__GET_LED_STATUS__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "mecanumbot_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "mecanumbot_msgs/srv/detail/get_led_status__struct.h"

/// Initialize srv/GetLedStatus message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * mecanumbot_msgs__srv__GetLedStatus_Request
 * )) before or use
 * mecanumbot_msgs__srv__GetLedStatus_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_mecanumbot_msgs
bool
mecanumbot_msgs__srv__GetLedStatus_Request__init(mecanumbot_msgs__srv__GetLedStatus_Request * msg);

/// Finalize srv/GetLedStatus message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_mecanumbot_msgs
void
mecanumbot_msgs__srv__GetLedStatus_Request__fini(mecanumbot_msgs__srv__GetLedStatus_Request * msg);

/// Create srv/GetLedStatus message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * mecanumbot_msgs__srv__GetLedStatus_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_mecanumbot_msgs
mecanumbot_msgs__srv__GetLedStatus_Request *
mecanumbot_msgs__srv__GetLedStatus_Request__create();

/// Destroy srv/GetLedStatus message.
/**
 * It calls
 * mecanumbot_msgs__srv__GetLedStatus_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_mecanumbot_msgs
void
mecanumbot_msgs__srv__GetLedStatus_Request__destroy(mecanumbot_msgs__srv__GetLedStatus_Request * msg);

/// Check for srv/GetLedStatus message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_mecanumbot_msgs
bool
mecanumbot_msgs__srv__GetLedStatus_Request__are_equal(const mecanumbot_msgs__srv__GetLedStatus_Request * lhs, const mecanumbot_msgs__srv__GetLedStatus_Request * rhs);

/// Copy a srv/GetLedStatus message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_mecanumbot_msgs
bool
mecanumbot_msgs__srv__GetLedStatus_Request__copy(
  const mecanumbot_msgs__srv__GetLedStatus_Request * input,
  mecanumbot_msgs__srv__GetLedStatus_Request * output);

/// Initialize array of srv/GetLedStatus messages.
/**
 * It allocates the memory for the number of elements and calls
 * mecanumbot_msgs__srv__GetLedStatus_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_mecanumbot_msgs
bool
mecanumbot_msgs__srv__GetLedStatus_Request__Sequence__init(mecanumbot_msgs__srv__GetLedStatus_Request__Sequence * array, size_t size);

/// Finalize array of srv/GetLedStatus messages.
/**
 * It calls
 * mecanumbot_msgs__srv__GetLedStatus_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_mecanumbot_msgs
void
mecanumbot_msgs__srv__GetLedStatus_Request__Sequence__fini(mecanumbot_msgs__srv__GetLedStatus_Request__Sequence * array);

/// Create array of srv/GetLedStatus messages.
/**
 * It allocates the memory for the array and calls
 * mecanumbot_msgs__srv__GetLedStatus_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_mecanumbot_msgs
mecanumbot_msgs__srv__GetLedStatus_Request__Sequence *
mecanumbot_msgs__srv__GetLedStatus_Request__Sequence__create(size_t size);

/// Destroy array of srv/GetLedStatus messages.
/**
 * It calls
 * mecanumbot_msgs__srv__GetLedStatus_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_mecanumbot_msgs
void
mecanumbot_msgs__srv__GetLedStatus_Request__Sequence__destroy(mecanumbot_msgs__srv__GetLedStatus_Request__Sequence * array);

/// Check for srv/GetLedStatus message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_mecanumbot_msgs
bool
mecanumbot_msgs__srv__GetLedStatus_Request__Sequence__are_equal(const mecanumbot_msgs__srv__GetLedStatus_Request__Sequence * lhs, const mecanumbot_msgs__srv__GetLedStatus_Request__Sequence * rhs);

/// Copy an array of srv/GetLedStatus messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_mecanumbot_msgs
bool
mecanumbot_msgs__srv__GetLedStatus_Request__Sequence__copy(
  const mecanumbot_msgs__srv__GetLedStatus_Request__Sequence * input,
  mecanumbot_msgs__srv__GetLedStatus_Request__Sequence * output);

/// Initialize srv/GetLedStatus message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * mecanumbot_msgs__srv__GetLedStatus_Response
 * )) before or use
 * mecanumbot_msgs__srv__GetLedStatus_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_mecanumbot_msgs
bool
mecanumbot_msgs__srv__GetLedStatus_Response__init(mecanumbot_msgs__srv__GetLedStatus_Response * msg);

/// Finalize srv/GetLedStatus message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_mecanumbot_msgs
void
mecanumbot_msgs__srv__GetLedStatus_Response__fini(mecanumbot_msgs__srv__GetLedStatus_Response * msg);

/// Create srv/GetLedStatus message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * mecanumbot_msgs__srv__GetLedStatus_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_mecanumbot_msgs
mecanumbot_msgs__srv__GetLedStatus_Response *
mecanumbot_msgs__srv__GetLedStatus_Response__create();

/// Destroy srv/GetLedStatus message.
/**
 * It calls
 * mecanumbot_msgs__srv__GetLedStatus_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_mecanumbot_msgs
void
mecanumbot_msgs__srv__GetLedStatus_Response__destroy(mecanumbot_msgs__srv__GetLedStatus_Response * msg);

/// Check for srv/GetLedStatus message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_mecanumbot_msgs
bool
mecanumbot_msgs__srv__GetLedStatus_Response__are_equal(const mecanumbot_msgs__srv__GetLedStatus_Response * lhs, const mecanumbot_msgs__srv__GetLedStatus_Response * rhs);

/// Copy a srv/GetLedStatus message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_mecanumbot_msgs
bool
mecanumbot_msgs__srv__GetLedStatus_Response__copy(
  const mecanumbot_msgs__srv__GetLedStatus_Response * input,
  mecanumbot_msgs__srv__GetLedStatus_Response * output);

/// Initialize array of srv/GetLedStatus messages.
/**
 * It allocates the memory for the number of elements and calls
 * mecanumbot_msgs__srv__GetLedStatus_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_mecanumbot_msgs
bool
mecanumbot_msgs__srv__GetLedStatus_Response__Sequence__init(mecanumbot_msgs__srv__GetLedStatus_Response__Sequence * array, size_t size);

/// Finalize array of srv/GetLedStatus messages.
/**
 * It calls
 * mecanumbot_msgs__srv__GetLedStatus_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_mecanumbot_msgs
void
mecanumbot_msgs__srv__GetLedStatus_Response__Sequence__fini(mecanumbot_msgs__srv__GetLedStatus_Response__Sequence * array);

/// Create array of srv/GetLedStatus messages.
/**
 * It allocates the memory for the array and calls
 * mecanumbot_msgs__srv__GetLedStatus_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_mecanumbot_msgs
mecanumbot_msgs__srv__GetLedStatus_Response__Sequence *
mecanumbot_msgs__srv__GetLedStatus_Response__Sequence__create(size_t size);

/// Destroy array of srv/GetLedStatus messages.
/**
 * It calls
 * mecanumbot_msgs__srv__GetLedStatus_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_mecanumbot_msgs
void
mecanumbot_msgs__srv__GetLedStatus_Response__Sequence__destroy(mecanumbot_msgs__srv__GetLedStatus_Response__Sequence * array);

/// Check for srv/GetLedStatus message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_mecanumbot_msgs
bool
mecanumbot_msgs__srv__GetLedStatus_Response__Sequence__are_equal(const mecanumbot_msgs__srv__GetLedStatus_Response__Sequence * lhs, const mecanumbot_msgs__srv__GetLedStatus_Response__Sequence * rhs);

/// Copy an array of srv/GetLedStatus messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_mecanumbot_msgs
bool
mecanumbot_msgs__srv__GetLedStatus_Response__Sequence__copy(
  const mecanumbot_msgs__srv__GetLedStatus_Response__Sequence * input,
  mecanumbot_msgs__srv__GetLedStatus_Response__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // MECANUMBOT_MSGS__SRV__DETAIL__GET_LED_STATUS__FUNCTIONS_H_
