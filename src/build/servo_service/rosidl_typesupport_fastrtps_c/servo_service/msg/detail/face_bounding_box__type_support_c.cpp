// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from servo_service:msg/FaceBoundingBox.idl
// generated code does not contain a copyright notice
#include "servo_service/msg/detail/face_bounding_box__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <cstddef>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/serialization_helpers.hpp"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "servo_service/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "servo_service/msg/detail/face_bounding_box__struct.h"
#include "servo_service/msg/detail/face_bounding_box__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif


// forward declare type support functions


using _FaceBoundingBox__ros_msg_type = servo_service__msg__FaceBoundingBox;


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_servo_service
bool cdr_serialize_servo_service__msg__FaceBoundingBox(
  const servo_service__msg__FaceBoundingBox * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: xmin
  {
    cdr << ros_message->xmin;
  }

  // Field name: ymin
  {
    cdr << ros_message->ymin;
  }

  // Field name: width
  {
    cdr << ros_message->width;
  }

  // Field name: height
  {
    cdr << ros_message->height;
  }

  // Field name: score
  {
    cdr << ros_message->score;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_servo_service
bool cdr_deserialize_servo_service__msg__FaceBoundingBox(
  eprosima::fastcdr::Cdr & cdr,
  servo_service__msg__FaceBoundingBox * ros_message)
{
  // Field name: xmin
  {
    cdr >> ros_message->xmin;
  }

  // Field name: ymin
  {
    cdr >> ros_message->ymin;
  }

  // Field name: width
  {
    cdr >> ros_message->width;
  }

  // Field name: height
  {
    cdr >> ros_message->height;
  }

  // Field name: score
  {
    cdr >> ros_message->score;
  }

  return true;
}  // NOLINT(readability/fn_size)


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_servo_service
size_t get_serialized_size_servo_service__msg__FaceBoundingBox(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _FaceBoundingBox__ros_msg_type * ros_message = static_cast<const _FaceBoundingBox__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: xmin
  {
    size_t item_size = sizeof(ros_message->xmin);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: ymin
  {
    size_t item_size = sizeof(ros_message->ymin);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: width
  {
    size_t item_size = sizeof(ros_message->width);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: height
  {
    size_t item_size = sizeof(ros_message->height);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: score
  {
    size_t item_size = sizeof(ros_message->score);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_servo_service
size_t max_serialized_size_servo_service__msg__FaceBoundingBox(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // Field name: xmin
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: ymin
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: width
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: height
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: score
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }


  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = servo_service__msg__FaceBoundingBox;
    is_plain =
      (
      offsetof(DataType, score) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_servo_service
bool cdr_serialize_key_servo_service__msg__FaceBoundingBox(
  const servo_service__msg__FaceBoundingBox * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: xmin
  {
    cdr << ros_message->xmin;
  }

  // Field name: ymin
  {
    cdr << ros_message->ymin;
  }

  // Field name: width
  {
    cdr << ros_message->width;
  }

  // Field name: height
  {
    cdr << ros_message->height;
  }

  // Field name: score
  {
    cdr << ros_message->score;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_servo_service
size_t get_serialized_size_key_servo_service__msg__FaceBoundingBox(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _FaceBoundingBox__ros_msg_type * ros_message = static_cast<const _FaceBoundingBox__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;

  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: xmin
  {
    size_t item_size = sizeof(ros_message->xmin);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: ymin
  {
    size_t item_size = sizeof(ros_message->ymin);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: width
  {
    size_t item_size = sizeof(ros_message->width);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: height
  {
    size_t item_size = sizeof(ros_message->height);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: score
  {
    size_t item_size = sizeof(ros_message->score);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_servo_service
size_t max_serialized_size_key_servo_service__msg__FaceBoundingBox(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;
  // Field name: xmin
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: ymin
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: width
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: height
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: score
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = servo_service__msg__FaceBoundingBox;
    is_plain =
      (
      offsetof(DataType, score) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}


static bool _FaceBoundingBox__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const servo_service__msg__FaceBoundingBox * ros_message = static_cast<const servo_service__msg__FaceBoundingBox *>(untyped_ros_message);
  (void)ros_message;
  return cdr_serialize_servo_service__msg__FaceBoundingBox(ros_message, cdr);
}

static bool _FaceBoundingBox__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  servo_service__msg__FaceBoundingBox * ros_message = static_cast<servo_service__msg__FaceBoundingBox *>(untyped_ros_message);
  (void)ros_message;
  return cdr_deserialize_servo_service__msg__FaceBoundingBox(cdr, ros_message);
}

static uint32_t _FaceBoundingBox__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_servo_service__msg__FaceBoundingBox(
      untyped_ros_message, 0));
}

static size_t _FaceBoundingBox__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_servo_service__msg__FaceBoundingBox(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_FaceBoundingBox = {
  "servo_service::msg",
  "FaceBoundingBox",
  _FaceBoundingBox__cdr_serialize,
  _FaceBoundingBox__cdr_deserialize,
  _FaceBoundingBox__get_serialized_size,
  _FaceBoundingBox__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _FaceBoundingBox__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_FaceBoundingBox,
  get_message_typesupport_handle_function,
  &servo_service__msg__FaceBoundingBox__get_type_hash,
  &servo_service__msg__FaceBoundingBox__get_type_description,
  &servo_service__msg__FaceBoundingBox__get_type_description_sources,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, servo_service, msg, FaceBoundingBox)() {
  return &_FaceBoundingBox__type_support;
}

#if defined(__cplusplus)
}
#endif
