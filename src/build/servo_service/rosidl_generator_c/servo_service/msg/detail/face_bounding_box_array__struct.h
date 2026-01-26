// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from servo_service:msg/FaceBoundingBoxArray.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "servo_service/msg/face_bounding_box_array.h"


#ifndef SERVO_SERVICE__MSG__DETAIL__FACE_BOUNDING_BOX_ARRAY__STRUCT_H_
#define SERVO_SERVICE__MSG__DETAIL__FACE_BOUNDING_BOX_ARRAY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'boxes'
#include "servo_service/msg/detail/face_bounding_box__struct.h"

/// Struct defined in msg/FaceBoundingBoxArray in the package servo_service.
typedef struct servo_service__msg__FaceBoundingBoxArray
{
  std_msgs__msg__Header header;
  servo_service__msg__FaceBoundingBox__Sequence boxes;
} servo_service__msg__FaceBoundingBoxArray;

// Struct for a sequence of servo_service__msg__FaceBoundingBoxArray.
typedef struct servo_service__msg__FaceBoundingBoxArray__Sequence
{
  servo_service__msg__FaceBoundingBoxArray * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} servo_service__msg__FaceBoundingBoxArray__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SERVO_SERVICE__MSG__DETAIL__FACE_BOUNDING_BOX_ARRAY__STRUCT_H_
