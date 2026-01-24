// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from servo_service:msg/FaceBoundingBox.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "servo_service/msg/face_bounding_box.h"


#ifndef SERVO_SERVICE__MSG__DETAIL__FACE_BOUNDING_BOX__STRUCT_H_
#define SERVO_SERVICE__MSG__DETAIL__FACE_BOUNDING_BOX__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

/// Struct defined in msg/FaceBoundingBox in the package servo_service.
typedef struct servo_service__msg__FaceBoundingBox
{
  float xmin;
  float ymin;
  float width;
  float height;
  float score;
} servo_service__msg__FaceBoundingBox;

// Struct for a sequence of servo_service__msg__FaceBoundingBox.
typedef struct servo_service__msg__FaceBoundingBox__Sequence
{
  servo_service__msg__FaceBoundingBox * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} servo_service__msg__FaceBoundingBox__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SERVO_SERVICE__MSG__DETAIL__FACE_BOUNDING_BOX__STRUCT_H_
