// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from servo_service:msg/FaceBoundingBox.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "servo_service/msg/face_bounding_box.hpp"


#ifndef SERVO_SERVICE__MSG__DETAIL__FACE_BOUNDING_BOX__TRAITS_HPP_
#define SERVO_SERVICE__MSG__DETAIL__FACE_BOUNDING_BOX__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "servo_service/msg/detail/face_bounding_box__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace servo_service
{

namespace msg
{

inline void to_flow_style_yaml(
  const FaceBoundingBox & msg,
  std::ostream & out)
{
  out << "{";
  // member: xmin
  {
    out << "xmin: ";
    rosidl_generator_traits::value_to_yaml(msg.xmin, out);
    out << ", ";
  }

  // member: ymin
  {
    out << "ymin: ";
    rosidl_generator_traits::value_to_yaml(msg.ymin, out);
    out << ", ";
  }

  // member: width
  {
    out << "width: ";
    rosidl_generator_traits::value_to_yaml(msg.width, out);
    out << ", ";
  }

  // member: height
  {
    out << "height: ";
    rosidl_generator_traits::value_to_yaml(msg.height, out);
    out << ", ";
  }

  // member: score
  {
    out << "score: ";
    rosidl_generator_traits::value_to_yaml(msg.score, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const FaceBoundingBox & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: xmin
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "xmin: ";
    rosidl_generator_traits::value_to_yaml(msg.xmin, out);
    out << "\n";
  }

  // member: ymin
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ymin: ";
    rosidl_generator_traits::value_to_yaml(msg.ymin, out);
    out << "\n";
  }

  // member: width
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "width: ";
    rosidl_generator_traits::value_to_yaml(msg.width, out);
    out << "\n";
  }

  // member: height
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "height: ";
    rosidl_generator_traits::value_to_yaml(msg.height, out);
    out << "\n";
  }

  // member: score
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "score: ";
    rosidl_generator_traits::value_to_yaml(msg.score, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const FaceBoundingBox & msg, bool use_flow_style = false)
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
  const servo_service::msg::FaceBoundingBox & msg,
  std::ostream & out, size_t indentation = 0)
{
  servo_service::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use servo_service::msg::to_yaml() instead")]]
inline std::string to_yaml(const servo_service::msg::FaceBoundingBox & msg)
{
  return servo_service::msg::to_yaml(msg);
}

template<>
inline const char * data_type<servo_service::msg::FaceBoundingBox>()
{
  return "servo_service::msg::FaceBoundingBox";
}

template<>
inline const char * name<servo_service::msg::FaceBoundingBox>()
{
  return "servo_service/msg/FaceBoundingBox";
}

template<>
struct has_fixed_size<servo_service::msg::FaceBoundingBox>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<servo_service::msg::FaceBoundingBox>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<servo_service::msg::FaceBoundingBox>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // SERVO_SERVICE__MSG__DETAIL__FACE_BOUNDING_BOX__TRAITS_HPP_
