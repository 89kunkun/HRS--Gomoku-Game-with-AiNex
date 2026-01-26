// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from servo_service:msg/FaceBoundingBoxArray.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "servo_service/msg/detail/face_bounding_box_array__functions.h"
#include "servo_service/msg/detail/face_bounding_box_array__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace servo_service
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void FaceBoundingBoxArray_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) servo_service::msg::FaceBoundingBoxArray(_init);
}

void FaceBoundingBoxArray_fini_function(void * message_memory)
{
  auto typed_message = static_cast<servo_service::msg::FaceBoundingBoxArray *>(message_memory);
  typed_message->~FaceBoundingBoxArray();
}

size_t size_function__FaceBoundingBoxArray__boxes(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<servo_service::msg::FaceBoundingBox> *>(untyped_member);
  return member->size();
}

const void * get_const_function__FaceBoundingBoxArray__boxes(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<servo_service::msg::FaceBoundingBox> *>(untyped_member);
  return &member[index];
}

void * get_function__FaceBoundingBoxArray__boxes(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<servo_service::msg::FaceBoundingBox> *>(untyped_member);
  return &member[index];
}

void fetch_function__FaceBoundingBoxArray__boxes(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const servo_service::msg::FaceBoundingBox *>(
    get_const_function__FaceBoundingBoxArray__boxes(untyped_member, index));
  auto & value = *reinterpret_cast<servo_service::msg::FaceBoundingBox *>(untyped_value);
  value = item;
}

void assign_function__FaceBoundingBoxArray__boxes(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<servo_service::msg::FaceBoundingBox *>(
    get_function__FaceBoundingBoxArray__boxes(untyped_member, index));
  const auto & value = *reinterpret_cast<const servo_service::msg::FaceBoundingBox *>(untyped_value);
  item = value;
}

void resize_function__FaceBoundingBoxArray__boxes(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<servo_service::msg::FaceBoundingBox> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember FaceBoundingBoxArray_message_member_array[2] = {
  {
    "header",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Header>(),  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(servo_service::msg::FaceBoundingBoxArray, header),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "boxes",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<servo_service::msg::FaceBoundingBox>(),  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(servo_service::msg::FaceBoundingBoxArray, boxes),  // bytes offset in struct
    nullptr,  // default value
    size_function__FaceBoundingBoxArray__boxes,  // size() function pointer
    get_const_function__FaceBoundingBoxArray__boxes,  // get_const(index) function pointer
    get_function__FaceBoundingBoxArray__boxes,  // get(index) function pointer
    fetch_function__FaceBoundingBoxArray__boxes,  // fetch(index, &value) function pointer
    assign_function__FaceBoundingBoxArray__boxes,  // assign(index, value) function pointer
    resize_function__FaceBoundingBoxArray__boxes  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers FaceBoundingBoxArray_message_members = {
  "servo_service::msg",  // message namespace
  "FaceBoundingBoxArray",  // message name
  2,  // number of fields
  sizeof(servo_service::msg::FaceBoundingBoxArray),
  false,  // has_any_key_member_
  FaceBoundingBoxArray_message_member_array,  // message members
  FaceBoundingBoxArray_init_function,  // function to initialize message memory (memory has to be allocated)
  FaceBoundingBoxArray_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t FaceBoundingBoxArray_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &FaceBoundingBoxArray_message_members,
  get_message_typesupport_handle_function,
  &servo_service__msg__FaceBoundingBoxArray__get_type_hash,
  &servo_service__msg__FaceBoundingBoxArray__get_type_description,
  &servo_service__msg__FaceBoundingBoxArray__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace servo_service


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<servo_service::msg::FaceBoundingBoxArray>()
{
  return &::servo_service::msg::rosidl_typesupport_introspection_cpp::FaceBoundingBoxArray_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, servo_service, msg, FaceBoundingBoxArray)() {
  return &::servo_service::msg::rosidl_typesupport_introspection_cpp::FaceBoundingBoxArray_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
