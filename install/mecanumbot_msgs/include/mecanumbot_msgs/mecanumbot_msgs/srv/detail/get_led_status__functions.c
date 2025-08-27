// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from mecanumbot_msgs:srv/GetLedStatus.idl
// generated code does not contain a copyright notice
#include "mecanumbot_msgs/srv/detail/get_led_status__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

bool
mecanumbot_msgs__srv__GetLedStatus_Request__init(mecanumbot_msgs__srv__GetLedStatus_Request * msg)
{
  if (!msg) {
    return false;
  }
  // structure_needs_at_least_one_member
  return true;
}

void
mecanumbot_msgs__srv__GetLedStatus_Request__fini(mecanumbot_msgs__srv__GetLedStatus_Request * msg)
{
  if (!msg) {
    return;
  }
  // structure_needs_at_least_one_member
}

bool
mecanumbot_msgs__srv__GetLedStatus_Request__are_equal(const mecanumbot_msgs__srv__GetLedStatus_Request * lhs, const mecanumbot_msgs__srv__GetLedStatus_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // structure_needs_at_least_one_member
  if (lhs->structure_needs_at_least_one_member != rhs->structure_needs_at_least_one_member) {
    return false;
  }
  return true;
}

bool
mecanumbot_msgs__srv__GetLedStatus_Request__copy(
  const mecanumbot_msgs__srv__GetLedStatus_Request * input,
  mecanumbot_msgs__srv__GetLedStatus_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // structure_needs_at_least_one_member
  output->structure_needs_at_least_one_member = input->structure_needs_at_least_one_member;
  return true;
}

mecanumbot_msgs__srv__GetLedStatus_Request *
mecanumbot_msgs__srv__GetLedStatus_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mecanumbot_msgs__srv__GetLedStatus_Request * msg = (mecanumbot_msgs__srv__GetLedStatus_Request *)allocator.allocate(sizeof(mecanumbot_msgs__srv__GetLedStatus_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mecanumbot_msgs__srv__GetLedStatus_Request));
  bool success = mecanumbot_msgs__srv__GetLedStatus_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mecanumbot_msgs__srv__GetLedStatus_Request__destroy(mecanumbot_msgs__srv__GetLedStatus_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mecanumbot_msgs__srv__GetLedStatus_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mecanumbot_msgs__srv__GetLedStatus_Request__Sequence__init(mecanumbot_msgs__srv__GetLedStatus_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mecanumbot_msgs__srv__GetLedStatus_Request * data = NULL;

  if (size) {
    data = (mecanumbot_msgs__srv__GetLedStatus_Request *)allocator.zero_allocate(size, sizeof(mecanumbot_msgs__srv__GetLedStatus_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mecanumbot_msgs__srv__GetLedStatus_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mecanumbot_msgs__srv__GetLedStatus_Request__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
mecanumbot_msgs__srv__GetLedStatus_Request__Sequence__fini(mecanumbot_msgs__srv__GetLedStatus_Request__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      mecanumbot_msgs__srv__GetLedStatus_Request__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

mecanumbot_msgs__srv__GetLedStatus_Request__Sequence *
mecanumbot_msgs__srv__GetLedStatus_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mecanumbot_msgs__srv__GetLedStatus_Request__Sequence * array = (mecanumbot_msgs__srv__GetLedStatus_Request__Sequence *)allocator.allocate(sizeof(mecanumbot_msgs__srv__GetLedStatus_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mecanumbot_msgs__srv__GetLedStatus_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mecanumbot_msgs__srv__GetLedStatus_Request__Sequence__destroy(mecanumbot_msgs__srv__GetLedStatus_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mecanumbot_msgs__srv__GetLedStatus_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mecanumbot_msgs__srv__GetLedStatus_Request__Sequence__are_equal(const mecanumbot_msgs__srv__GetLedStatus_Request__Sequence * lhs, const mecanumbot_msgs__srv__GetLedStatus_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mecanumbot_msgs__srv__GetLedStatus_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mecanumbot_msgs__srv__GetLedStatus_Request__Sequence__copy(
  const mecanumbot_msgs__srv__GetLedStatus_Request__Sequence * input,
  mecanumbot_msgs__srv__GetLedStatus_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mecanumbot_msgs__srv__GetLedStatus_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    mecanumbot_msgs__srv__GetLedStatus_Request * data =
      (mecanumbot_msgs__srv__GetLedStatus_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mecanumbot_msgs__srv__GetLedStatus_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          mecanumbot_msgs__srv__GetLedStatus_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mecanumbot_msgs__srv__GetLedStatus_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
mecanumbot_msgs__srv__GetLedStatus_Response__init(mecanumbot_msgs__srv__GetLedStatus_Response * msg)
{
  if (!msg) {
    return false;
  }
  // fl_mode
  // fl_color
  // fr_mode
  // fr_color
  // br_mode
  // br_color
  // bl_mode
  // bl_color
  return true;
}

void
mecanumbot_msgs__srv__GetLedStatus_Response__fini(mecanumbot_msgs__srv__GetLedStatus_Response * msg)
{
  if (!msg) {
    return;
  }
  // fl_mode
  // fl_color
  // fr_mode
  // fr_color
  // br_mode
  // br_color
  // bl_mode
  // bl_color
}

bool
mecanumbot_msgs__srv__GetLedStatus_Response__are_equal(const mecanumbot_msgs__srv__GetLedStatus_Response * lhs, const mecanumbot_msgs__srv__GetLedStatus_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // fl_mode
  if (lhs->fl_mode != rhs->fl_mode) {
    return false;
  }
  // fl_color
  if (lhs->fl_color != rhs->fl_color) {
    return false;
  }
  // fr_mode
  if (lhs->fr_mode != rhs->fr_mode) {
    return false;
  }
  // fr_color
  if (lhs->fr_color != rhs->fr_color) {
    return false;
  }
  // br_mode
  if (lhs->br_mode != rhs->br_mode) {
    return false;
  }
  // br_color
  if (lhs->br_color != rhs->br_color) {
    return false;
  }
  // bl_mode
  if (lhs->bl_mode != rhs->bl_mode) {
    return false;
  }
  // bl_color
  if (lhs->bl_color != rhs->bl_color) {
    return false;
  }
  return true;
}

bool
mecanumbot_msgs__srv__GetLedStatus_Response__copy(
  const mecanumbot_msgs__srv__GetLedStatus_Response * input,
  mecanumbot_msgs__srv__GetLedStatus_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // fl_mode
  output->fl_mode = input->fl_mode;
  // fl_color
  output->fl_color = input->fl_color;
  // fr_mode
  output->fr_mode = input->fr_mode;
  // fr_color
  output->fr_color = input->fr_color;
  // br_mode
  output->br_mode = input->br_mode;
  // br_color
  output->br_color = input->br_color;
  // bl_mode
  output->bl_mode = input->bl_mode;
  // bl_color
  output->bl_color = input->bl_color;
  return true;
}

mecanumbot_msgs__srv__GetLedStatus_Response *
mecanumbot_msgs__srv__GetLedStatus_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mecanumbot_msgs__srv__GetLedStatus_Response * msg = (mecanumbot_msgs__srv__GetLedStatus_Response *)allocator.allocate(sizeof(mecanumbot_msgs__srv__GetLedStatus_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mecanumbot_msgs__srv__GetLedStatus_Response));
  bool success = mecanumbot_msgs__srv__GetLedStatus_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mecanumbot_msgs__srv__GetLedStatus_Response__destroy(mecanumbot_msgs__srv__GetLedStatus_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mecanumbot_msgs__srv__GetLedStatus_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mecanumbot_msgs__srv__GetLedStatus_Response__Sequence__init(mecanumbot_msgs__srv__GetLedStatus_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mecanumbot_msgs__srv__GetLedStatus_Response * data = NULL;

  if (size) {
    data = (mecanumbot_msgs__srv__GetLedStatus_Response *)allocator.zero_allocate(size, sizeof(mecanumbot_msgs__srv__GetLedStatus_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mecanumbot_msgs__srv__GetLedStatus_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mecanumbot_msgs__srv__GetLedStatus_Response__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
mecanumbot_msgs__srv__GetLedStatus_Response__Sequence__fini(mecanumbot_msgs__srv__GetLedStatus_Response__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      mecanumbot_msgs__srv__GetLedStatus_Response__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

mecanumbot_msgs__srv__GetLedStatus_Response__Sequence *
mecanumbot_msgs__srv__GetLedStatus_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mecanumbot_msgs__srv__GetLedStatus_Response__Sequence * array = (mecanumbot_msgs__srv__GetLedStatus_Response__Sequence *)allocator.allocate(sizeof(mecanumbot_msgs__srv__GetLedStatus_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mecanumbot_msgs__srv__GetLedStatus_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mecanumbot_msgs__srv__GetLedStatus_Response__Sequence__destroy(mecanumbot_msgs__srv__GetLedStatus_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mecanumbot_msgs__srv__GetLedStatus_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mecanumbot_msgs__srv__GetLedStatus_Response__Sequence__are_equal(const mecanumbot_msgs__srv__GetLedStatus_Response__Sequence * lhs, const mecanumbot_msgs__srv__GetLedStatus_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mecanumbot_msgs__srv__GetLedStatus_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mecanumbot_msgs__srv__GetLedStatus_Response__Sequence__copy(
  const mecanumbot_msgs__srv__GetLedStatus_Response__Sequence * input,
  mecanumbot_msgs__srv__GetLedStatus_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mecanumbot_msgs__srv__GetLedStatus_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    mecanumbot_msgs__srv__GetLedStatus_Response * data =
      (mecanumbot_msgs__srv__GetLedStatus_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mecanumbot_msgs__srv__GetLedStatus_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          mecanumbot_msgs__srv__GetLedStatus_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mecanumbot_msgs__srv__GetLedStatus_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
