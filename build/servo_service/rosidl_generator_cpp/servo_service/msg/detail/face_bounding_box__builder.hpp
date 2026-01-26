// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from servo_service:msg/FaceBoundingBox.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "servo_service/msg/face_bounding_box.hpp"


#ifndef SERVO_SERVICE__MSG__DETAIL__FACE_BOUNDING_BOX__BUILDER_HPP_
#define SERVO_SERVICE__MSG__DETAIL__FACE_BOUNDING_BOX__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "servo_service/msg/detail/face_bounding_box__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace servo_service
{

namespace msg
{

namespace builder
{

class Init_FaceBoundingBox_score
{
public:
  explicit Init_FaceBoundingBox_score(::servo_service::msg::FaceBoundingBox & msg)
  : msg_(msg)
  {}
  ::servo_service::msg::FaceBoundingBox score(::servo_service::msg::FaceBoundingBox::_score_type arg)
  {
    msg_.score = std::move(arg);
    return std::move(msg_);
  }

private:
  ::servo_service::msg::FaceBoundingBox msg_;
};

class Init_FaceBoundingBox_height
{
public:
  explicit Init_FaceBoundingBox_height(::servo_service::msg::FaceBoundingBox & msg)
  : msg_(msg)
  {}
  Init_FaceBoundingBox_score height(::servo_service::msg::FaceBoundingBox::_height_type arg)
  {
    msg_.height = std::move(arg);
    return Init_FaceBoundingBox_score(msg_);
  }

private:
  ::servo_service::msg::FaceBoundingBox msg_;
};

class Init_FaceBoundingBox_width
{
public:
  explicit Init_FaceBoundingBox_width(::servo_service::msg::FaceBoundingBox & msg)
  : msg_(msg)
  {}
  Init_FaceBoundingBox_height width(::servo_service::msg::FaceBoundingBox::_width_type arg)
  {
    msg_.width = std::move(arg);
    return Init_FaceBoundingBox_height(msg_);
  }

private:
  ::servo_service::msg::FaceBoundingBox msg_;
};

class Init_FaceBoundingBox_ymin
{
public:
  explicit Init_FaceBoundingBox_ymin(::servo_service::msg::FaceBoundingBox & msg)
  : msg_(msg)
  {}
  Init_FaceBoundingBox_width ymin(::servo_service::msg::FaceBoundingBox::_ymin_type arg)
  {
    msg_.ymin = std::move(arg);
    return Init_FaceBoundingBox_width(msg_);
  }

private:
  ::servo_service::msg::FaceBoundingBox msg_;
};

class Init_FaceBoundingBox_xmin
{
public:
  Init_FaceBoundingBox_xmin()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_FaceBoundingBox_ymin xmin(::servo_service::msg::FaceBoundingBox::_xmin_type arg)
  {
    msg_.xmin = std::move(arg);
    return Init_FaceBoundingBox_ymin(msg_);
  }

private:
  ::servo_service::msg::FaceBoundingBox msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::servo_service::msg::FaceBoundingBox>()
{
  return servo_service::msg::builder::Init_FaceBoundingBox_xmin();
}

}  // namespace servo_service

#endif  // SERVO_SERVICE__MSG__DETAIL__FACE_BOUNDING_BOX__BUILDER_HPP_
