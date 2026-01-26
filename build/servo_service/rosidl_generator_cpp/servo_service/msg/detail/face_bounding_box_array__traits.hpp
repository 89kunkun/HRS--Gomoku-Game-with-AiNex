// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from servo_service:msg/FaceBoundingBoxArray.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "servo_service/msg/face_bounding_box_array.hpp"


#ifndef SERVO_SERVICE__MSG__DETAIL__FACE_BOUNDING_BOX_ARRAY__TRAITS_HPP_
#define SERVO_SERVICE__MSG__DETAIL__FACE_BOUNDING_BOX_ARRAY__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "servo_service/msg/detail/face_bounding_box_array__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'boxes'
#include "servo_service/msg/detail/face_bounding_box__traits.hpp"

namespace servo_service
{

namespace msg
{

inline void to_flow_style_yaml(
  const FaceBoundingBoxArray & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: boxes
  {
    if (msg.boxes.size() == 0) {
      out << "boxes: []";
    } else {
      out << "boxes: [";
      size_t pending_items = msg.boxes.size();
      for (auto item : msg.boxes) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const FaceBoundingBoxArray & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: boxes
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.boxes.size() == 0) {
      out << "boxes: []\n";
    } else {
      out << "boxes:\n";
      for (auto item : msg.boxes) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const FaceBoundingBoxArray & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace servo_service

namespace rosidl_generator_traits
{

[[deprecated("use servo_service::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const servo_service::msg::FaceBoundingBoxArray & msg,
  std::ostream & out, size_t indentation = 0)
{
  servo_service::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use servo_service::msg::to_yaml() instead")]]
inline std::string to_yaml(const servo_service::msg::FaceBoundingBoxArray & msg)
{
  return servo_service::msg::to_yaml(msg);
}

template<>
inline const char * data_type<servo_service::msg::FaceBoundingBoxArray>()
{
  return "servo_service::msg::FaceBoundingBoxArray";
}

template<>
inline const char * name<servo_service::msg::FaceBoundingBoxArray>()
{
  return "servo_service/msg/FaceBoundingBoxArray";
}

template<>
struct has_fixed_size<servo_service::msg::FaceBoundingBoxArray>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<servo_service::msg::FaceBoundingBoxArray>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<servo_service::msg::FaceBoundingBoxArray>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // SERVO_SERVICE__MSG__DETAIL__FACE_BOUNDING_BOX_ARRAY__TRAITS_HPP_
