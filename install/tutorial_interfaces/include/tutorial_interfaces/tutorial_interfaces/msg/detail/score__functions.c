// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from tutorial_interfaces:msg/Score.idl
// generated code does not contain a copyright notice
#include "tutorial_interfaces/msg/detail/score__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
tutorial_interfaces__msg__Score__init(tutorial_interfaces__msg__Score * msg)
{
  if (!msg) {
    return false;
  }
  // first
  // second
  return true;
}

void
tutorial_interfaces__msg__Score__fini(tutorial_interfaces__msg__Score * msg)
{
  if (!msg) {
    return;
  }
  // first
  // second
}

bool
tutorial_interfaces__msg__Score__are_equal(const tutorial_interfaces__msg__Score * lhs, const tutorial_interfaces__msg__Score * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // first
  if (lhs->first != rhs->first) {
    return false;
  }
  // second
  if (lhs->second != rhs->second) {
    return false;
  }
  return true;
}

bool
tutorial_interfaces__msg__Score__copy(
  const tutorial_interfaces__msg__Score * input,
  tutorial_interfaces__msg__Score * output)
{
  if (!input || !output) {
    return false;
  }
  // first
  output->first = input->first;
  // second
  output->second = input->second;
  return true;
}

tutorial_interfaces__msg__Score *
tutorial_interfaces__msg__Score__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tutorial_interfaces__msg__Score * msg = (tutorial_interfaces__msg__Score *)allocator.allocate(sizeof(tutorial_interfaces__msg__Score), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(tutorial_interfaces__msg__Score));
  bool success = tutorial_interfaces__msg__Score__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
tutorial_interfaces__msg__Score__destroy(tutorial_interfaces__msg__Score * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    tutorial_interfaces__msg__Score__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
tutorial_interfaces__msg__Score__Sequence__init(tutorial_interfaces__msg__Score__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tutorial_interfaces__msg__Score * data = NULL;

  if (size) {
    data = (tutorial_interfaces__msg__Score *)allocator.zero_allocate(size, sizeof(tutorial_interfaces__msg__Score), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = tutorial_interfaces__msg__Score__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        tutorial_interfaces__msg__Score__fini(&data[i - 1]);
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
tutorial_interfaces__msg__Score__Sequence__fini(tutorial_interfaces__msg__Score__Sequence * array)
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
      tutorial_interfaces__msg__Score__fini(&array->data[i]);
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

tutorial_interfaces__msg__Score__Sequence *
tutorial_interfaces__msg__Score__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tutorial_interfaces__msg__Score__Sequence * array = (tutorial_interfaces__msg__Score__Sequence *)allocator.allocate(sizeof(tutorial_interfaces__msg__Score__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = tutorial_interfaces__msg__Score__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
tutorial_interfaces__msg__Score__Sequence__destroy(tutorial_interfaces__msg__Score__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    tutorial_interfaces__msg__Score__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
tutorial_interfaces__msg__Score__Sequence__are_equal(const tutorial_interfaces__msg__Score__Sequence * lhs, const tutorial_interfaces__msg__Score__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!tutorial_interfaces__msg__Score__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
tutorial_interfaces__msg__Score__Sequence__copy(
  const tutorial_interfaces__msg__Score__Sequence * input,
  tutorial_interfaces__msg__Score__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(tutorial_interfaces__msg__Score);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    tutorial_interfaces__msg__Score * data =
      (tutorial_interfaces__msg__Score *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!tutorial_interfaces__msg__Score__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          tutorial_interfaces__msg__Score__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!tutorial_interfaces__msg__Score__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
