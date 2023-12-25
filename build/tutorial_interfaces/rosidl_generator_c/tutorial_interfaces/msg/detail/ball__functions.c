// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from tutorial_interfaces:msg/Ball.idl
// generated code does not contain a copyright notice
#include "tutorial_interfaces/msg/detail/ball__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
tutorial_interfaces__msg__Ball__init(tutorial_interfaces__msg__Ball * msg)
{
  if (!msg) {
    return false;
  }
  // x
  // y
  return true;
}

void
tutorial_interfaces__msg__Ball__fini(tutorial_interfaces__msg__Ball * msg)
{
  if (!msg) {
    return;
  }
  // x
  // y
}

bool
tutorial_interfaces__msg__Ball__are_equal(const tutorial_interfaces__msg__Ball * lhs, const tutorial_interfaces__msg__Ball * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // x
  if (lhs->x != rhs->x) {
    return false;
  }
  // y
  if (lhs->y != rhs->y) {
    return false;
  }
  return true;
}

bool
tutorial_interfaces__msg__Ball__copy(
  const tutorial_interfaces__msg__Ball * input,
  tutorial_interfaces__msg__Ball * output)
{
  if (!input || !output) {
    return false;
  }
  // x
  output->x = input->x;
  // y
  output->y = input->y;
  return true;
}

tutorial_interfaces__msg__Ball *
tutorial_interfaces__msg__Ball__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tutorial_interfaces__msg__Ball * msg = (tutorial_interfaces__msg__Ball *)allocator.allocate(sizeof(tutorial_interfaces__msg__Ball), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(tutorial_interfaces__msg__Ball));
  bool success = tutorial_interfaces__msg__Ball__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
tutorial_interfaces__msg__Ball__destroy(tutorial_interfaces__msg__Ball * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    tutorial_interfaces__msg__Ball__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
tutorial_interfaces__msg__Ball__Sequence__init(tutorial_interfaces__msg__Ball__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tutorial_interfaces__msg__Ball * data = NULL;

  if (size) {
    data = (tutorial_interfaces__msg__Ball *)allocator.zero_allocate(size, sizeof(tutorial_interfaces__msg__Ball), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = tutorial_interfaces__msg__Ball__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        tutorial_interfaces__msg__Ball__fini(&data[i - 1]);
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
tutorial_interfaces__msg__Ball__Sequence__fini(tutorial_interfaces__msg__Ball__Sequence * array)
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
      tutorial_interfaces__msg__Ball__fini(&array->data[i]);
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

tutorial_interfaces__msg__Ball__Sequence *
tutorial_interfaces__msg__Ball__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tutorial_interfaces__msg__Ball__Sequence * array = (tutorial_interfaces__msg__Ball__Sequence *)allocator.allocate(sizeof(tutorial_interfaces__msg__Ball__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = tutorial_interfaces__msg__Ball__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
tutorial_interfaces__msg__Ball__Sequence__destroy(tutorial_interfaces__msg__Ball__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    tutorial_interfaces__msg__Ball__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
tutorial_interfaces__msg__Ball__Sequence__are_equal(const tutorial_interfaces__msg__Ball__Sequence * lhs, const tutorial_interfaces__msg__Ball__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!tutorial_interfaces__msg__Ball__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
tutorial_interfaces__msg__Ball__Sequence__copy(
  const tutorial_interfaces__msg__Ball__Sequence * input,
  tutorial_interfaces__msg__Ball__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(tutorial_interfaces__msg__Ball);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    tutorial_interfaces__msg__Ball * data =
      (tutorial_interfaces__msg__Ball *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!tutorial_interfaces__msg__Ball__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          tutorial_interfaces__msg__Ball__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!tutorial_interfaces__msg__Ball__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
