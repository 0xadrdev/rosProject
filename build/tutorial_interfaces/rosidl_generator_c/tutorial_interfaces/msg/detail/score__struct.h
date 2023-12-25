// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from tutorial_interfaces:msg/Score.idl
// generated code does not contain a copyright notice

#ifndef TUTORIAL_INTERFACES__MSG__DETAIL__SCORE__STRUCT_H_
#define TUTORIAL_INTERFACES__MSG__DETAIL__SCORE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/Score in the package tutorial_interfaces.
typedef struct tutorial_interfaces__msg__Score
{
  int32_t first;
  int32_t second;
} tutorial_interfaces__msg__Score;

// Struct for a sequence of tutorial_interfaces__msg__Score.
typedef struct tutorial_interfaces__msg__Score__Sequence
{
  tutorial_interfaces__msg__Score * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} tutorial_interfaces__msg__Score__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // TUTORIAL_INTERFACES__MSG__DETAIL__SCORE__STRUCT_H_
