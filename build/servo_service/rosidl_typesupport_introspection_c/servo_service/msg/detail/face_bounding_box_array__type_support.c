// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from servo_service:msg/FaceBoundingBoxArray.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "servo_service/msg/detail/face_bounding_box_array__rosidl_typesupport_introspection_c.h"
#include "servo_service/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "servo_service/msg/detail/face_bounding_box_array__functions.h"
#include "servo_service/msg/detail/face_bounding_box_array__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `boxes`
#include "servo_service/msg/face_bounding_box.h"
// Member `boxes`
#include "servo_service/msg/detail/face_bounding_box__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void servo_service__msg__FaceBoundingBoxArray__rosidl_typesupport_introspection_c__FaceBoundingBoxArray_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  servo_service__msg__FaceBoundingBoxArray__init(message_memory);
}

void servo_service__msg__FaceBoundingBoxArray__rosidl_typesupport_introspection_c__FaceBoundingBoxArray_fini_function(void * message_memory)
{
  servo_service__msg__FaceBoundingBoxArray__fini(message_memory);
}

size_t servo_service__msg__FaceBoundingBoxArray__rosidl_typesupport_introspection_c__size_function__FaceBoundingBoxArray__boxes(
  const void * untyped_member)
{
  const servo_service__msg__FaceBoundingBox__Sequence * member =
    (const servo_service__msg__FaceBoundingBox__Sequence *)(untyped_member);
  return member->size;
}

const void * servo_service__msg__FaceBoundingBoxArray__rosidl_typesupport_introspection_c__get_const_function__FaceBoundingBoxArray__boxes(
  const void * untyped_member, size_t index)
{
  const servo_service__msg__FaceBoundingBox__Sequence * member =
    (const servo_service__msg__FaceBoundingBox__Sequence *)(untyped_member);
  return &member->data[index];
}

void * servo_service__msg__FaceBoundingBoxArray__rosidl_typesupport_introspection_c__get_function__FaceBoundingBoxArray__boxes(
  void * untyped_member, size_t index)
{
  servo_service__msg__FaceBoundingBox__Sequence * member =
    (servo_service__msg__FaceBoundingBox__Sequence *)(untyped_member);
  return &member->data[index];
}

void servo_service__msg__FaceBoundingBoxArray__rosidl_typesupport_introspection_c__fetch_function__FaceBoundingBoxArray__boxes(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const servo_service__msg__FaceBoundingBox * item =
    ((const servo_service__msg__FaceBoundingBox *)
    servo_service__msg__FaceBoundingBoxArray__rosidl_typesupport_introspection_c__get_const_function__FaceBoundingBoxArray__boxes(untyped_member, index));
  servo_service__msg__FaceBoundingBox * value =
    (servo_service__msg__FaceBoundingBox *)(untyped_value);
  *value = *item;
}

void servo_service__msg__FaceBoundingBoxArray__rosidl_typesupport_introspection_c__assign_function__FaceBoundingBoxArray__boxes(
  void * untyped_member, size_t index, const void * untyped_value)
{
  servo_service__msg__FaceBoundingBox * item =
    ((servo_service__msg__FaceBoundingBox *)
    servo_service__msg__FaceBoundingBoxArray__rosidl_typesupport_introspection_c__get_function__FaceBoundingBoxArray__boxes(untyped_member, index));
  const servo_service__msg__FaceBoundingBox * value =
    (const servo_service__msg__FaceBoundingBox *)(untyped_value);
  *item = *value;
}

bool servo_service__msg__FaceBoundingBoxArray__rosidl_typesupport_introspection_c__resize_function__FaceBoundingBoxArray__boxes(
  void * untyped_member, size_t size)
{
  servo_service__msg__FaceBoundingBox__Sequence * member =
    (servo_service__msg__FaceBoundingBox__Sequence *)(untyped_member);
  servo_service__msg__FaceBoundingBox__Sequence__fini(member);
  return servo_service__msg__FaceBoundingBox__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember servo_service__msg__FaceBoundingBoxArray__rosidl_typesupport_introspection_c__FaceBoundingBoxArray_message_member_array[2] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(servo_service__msg__FaceBoundingBoxArray, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "boxes",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(servo_service__msg__FaceBoundingBoxArray, boxes),  // bytes offset in struct
    NULL,  // default value
    servo_service__msg__FaceBoundingBoxArray__rosidl_typesupport_introspection_c__size_function__FaceBoundingBoxArray__boxes,  // size() function pointer
    servo_service__msg__FaceBoundingBoxArray__rosidl_typesupport_introspection_c__get_const_function__FaceBoundingBoxArray__boxes,  // get_const(index) function pointer
    servo_service__msg__FaceBoundingBoxArray__rosidl_typesupport_introspection_c__get_function__FaceBoundingBoxArray__boxes,  // get(index) function pointer
    servo_service__msg__FaceBoundingBoxArray__rosidl_typesupport_introspection_c__fetch_function__FaceBoundingBoxArray__boxes,  // fetch(index, &value) function pointer
    servo_service__msg__FaceBoundingBoxArray__rosidl_typesupport_introspection_c__assign_function__FaceBoundingBoxArray__boxes,  // assign(index, value) function pointer
    servo_service__msg__FaceBoundingBoxArray__rosidl_typesupport_introspection_c__resize_function__FaceBoundingBoxArray__boxes  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers servo_service__msg__FaceBoundingBoxArray__rosidl_typesupport_introspection_c__FaceBoundingBoxArray_message_members = {
  "servo_service__msg",  // message namespace
  "FaceBoundingBoxArray",  // message name
  2,  // number of fields
  sizeof(servo_service__msg__FaceBoundingBoxArray),
  false,  // has_any_key_member_
  servo_service__msg__FaceBoundingBoxArray__rosidl_typesupport_introspection_c__FaceBoundingBoxArray_message_member_array,  // message members
  servo_service__msg__FaceBoundingBoxArray__rosidl_typesupport_introspection_c__FaceBoundingBoxArray_init_function,  // function to initialize message memory (memory has to be allocated)
  servo_service__msg__FaceBoundingBoxArray__rosidl_typesupport_introspection_c__FaceBoundingBoxArray_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t servo_service__msg__FaceBoundingBoxArray__rosidl_typesupport_introspection_c__FaceBoundingBoxArray_message_type_support_handle = {
  0,
  &servo_service__msg__FaceBoundingBoxArray__rosidl_typesupport_introspection_c__FaceBoundingBoxArray_message_members,
  get_message_typesupport_handle_function,
  &servo_service__msg__FaceBoundingBoxArray__get_type_hash,
  &servo_service__msg__FaceBoundingBoxArray__get_type_description,
  &servo_service__msg__FaceBoundingBoxArray__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_servo_service
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, servo_service, msg, FaceBoundingBoxArray)() {
  servo_service__msg__FaceBoundingBoxArray__rosidl_typesupport_introspection_c__FaceBoundingBoxArray_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  servo_service__msg__FaceBoundingBoxArray__rosidl_typesupport_introspection_c__FaceBoundingBoxArray_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, servo_service, msg, FaceBoundingBox)();
  if (!servo_service__msg__FaceBoundingBoxArray__rosidl_typesupport_introspection_c__FaceBoundingBoxArray_message_type_support_handle.typesupport_identifier) {
    servo_service__msg__FaceBoundingBoxArray__rosidl_typesupport_introspection_c__FaceBoundingBoxArray_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &servo_service__msg__FaceBoundingBoxArray__rosidl_typesupport_introspection_c__FaceBoundingBoxArray_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
