// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from servo_service:msg/FaceBoundingBox.idl
// generated code does not contain a copyright notice
#include "servo_service/msg/detail/face_bounding_box__rosidl_typesupport_fastrtps_cpp.hpp"
#include "servo_service/msg/detail/face_bounding_box__functions.h"
#include "servo_service/msg/detail/face_bounding_box__struct.hpp"

#include <cstddef>
#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/serialization_helpers.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

namespace servo_service
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{


bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_servo_service
cdr_serialize(
  const servo_service::msg::FaceBoundingBox & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: xmin
  cdr << ros_message.xmin;

  // Member: ymin
  cdr << ros_message.ymin;

  // Member: width
  cdr << ros_message.width;

  // Member: height
  cdr << ros_message.height;

  // Member: score
  cdr << ros_message.score;

  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_servo_service
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  servo_service::msg::FaceBoundingBox & ros_message)
{
  // Member: xmin
  cdr >> ros_message.xmin;

  // Member: ymin
  cdr >> ros_message.ymin;

  // Member: width
  cdr >> ros_message.width;

  // Member: height
  cdr >> ros_message.height;

  // Member: score
  cdr >> ros_message.score;

  return true;
}


size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_servo_service
get_serialized_size(
  const servo_service::msg::FaceBoundingBox & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: xmin
  {
    size_t item_size = sizeof(ros_message.xmin);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: ymin
  {
    size_t item_size = sizeof(ros_message.ymin);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: width
  {
    size_t item_size = sizeof(ros_message.width);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: height
  {
    size_t item_size = sizeof(ros_message.height);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: score
  {
    size_t item_size = sizeof(ros_message.score);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}


size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_servo_service
max_serialized_size_FaceBoundingBox(
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

  // Member: xmin
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // Member: ymin
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // Member: width
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // Member: height
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // Member: score
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
    using DataType = servo_service::msg::FaceBoundingBox;
    is_plain =
      (
      offsetof(DataType, score) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_servo_service
cdr_serialize_key(
  const servo_service::msg::FaceBoundingBox & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: xmin
  cdr << ros_message.xmin;

  // Member: ymin
  cdr << ros_message.ymin;

  // Member: width
  cdr << ros_message.width;

  // Member: height
  cdr << ros_message.height;

  // Member: score
  cdr << ros_message.score;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_servo_service
get_serialized_size_key(
  const servo_service::msg::FaceBoundingBox & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: xmin
  {
    size_t item_size = sizeof(ros_message.xmin);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: ymin
  {
    size_t item_size = sizeof(ros_message.ymin);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: width
  {
    size_t item_size = sizeof(ros_message.width);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: height
  {
    size_t item_size = sizeof(ros_message.height);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: score
  {
    size_t item_size = sizeof(ros_message.score);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_servo_service
max_serialized_size_key_FaceBoundingBox(
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

  // Member: xmin
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: ymin
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: width
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: height
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: score
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
    using DataType = servo_service::msg::FaceBoundingBox;
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
  auto typed_message =
    static_cast<const servo_service::msg::FaceBoundingBox *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _FaceBoundingBox__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<servo_service::msg::FaceBoundingBox *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _FaceBoundingBox__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const servo_service::msg::FaceBoundingBox *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _FaceBoundingBox__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_FaceBoundingBox(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _FaceBoundingBox__callbacks = {
  "servo_service::msg",
  "FaceBoundingBox",
  _FaceBoundingBox__cdr_serialize,
  _FaceBoundingBox__cdr_deserialize,
  _FaceBoundingBox__get_serialized_size,
  _FaceBoundingBox__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _FaceBoundingBox__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_FaceBoundingBox__callbacks,
  get_message_typesupport_handle_function,
  &servo_service__msg__FaceBoundingBox__get_type_hash,
  &servo_service__msg__FaceBoundingBox__get_type_description,
  &servo_service__msg__FaceBoundingBox__get_type_description_sources,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace servo_service

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_servo_service
const rosidl_message_type_support_t *
get_message_type_support_handle<servo_service::msg::FaceBoundingBox>()
{
  return &servo_service::msg::typesupport_fastrtps_cpp::_FaceBoundingBox__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, servo_service, msg, FaceBoundingBox)() {
  return &servo_service::msg::typesupport_fastrtps_cpp::_FaceBoundingBox__handle;
}

#ifdef __cplusplus
}
#endif
