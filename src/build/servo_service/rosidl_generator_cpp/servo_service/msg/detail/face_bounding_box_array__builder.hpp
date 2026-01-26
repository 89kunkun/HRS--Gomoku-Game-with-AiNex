// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from servo_service:msg/FaceBoundingBoxArray.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "servo_service/msg/face_bounding_box_array.hpp"


#ifndef SERVO_SERVICE__MSG__DETAIL__FACE_BOUNDING_BOX_ARRAY__BUILDER_HPP_
#define SERVO_SERVICE__MSG__DETAIL__FACE_BOUNDING_BOX_ARRAY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "servo_service/msg/detail/face_bounding_box_array__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace servo_service
{

namespace msg
{

namespace builder
{

class Init_FaceBoundingBoxArray_boxes
{
public:
  explicit Init_FaceBoundingBoxArray_boxes(::servo_service::msg::FaceBoundingBoxArray & msg)
  : msg_(msg)
  {}
  ::servo_service::msg::FaceBoundingBoxArray boxes(::servo_service::msg::FaceBoundingBoxArray::_boxes_type arg)
  {
    msg_.boxes = std::move(arg);
    return std::move(msg_);
  }

private:
  ::servo_service::msg::FaceBoundingBoxArray msg_;
};

class Init_FaceBoundingBoxArray_header
{
public:
  Init_FaceBoundingBoxArray_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_FaceBoundingBoxArray_boxes header(::servo_service::msg::FaceBoundingBoxArray::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_FaceBoundingBoxArray_boxes(msg_);
  }

private:
  ::servo_service::msg::FaceBoundingBoxArray msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::servo_service::msg::FaceBoundingBoxArray>()
{
  return servo_service::msg::builder::Init_FaceBoundingBoxArray_header();
}

}  // namespace servo_service

#endif  // SERVO_SERVICE__MSG__DETAIL__FACE_BOUNDING_BOX_ARRAY__BUILDER_HPP_
