// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from servo_service:msg/FaceBoundingBox.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "servo_service/msg/face_bounding_box.hpp"


#ifndef SERVO_SERVICE__MSG__DETAIL__FACE_BOUNDING_BOX__STRUCT_HPP_
#define SERVO_SERVICE__MSG__DETAIL__FACE_BOUNDING_BOX__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__servo_service__msg__FaceBoundingBox __attribute__((deprecated))
#else
# define DEPRECATED__servo_service__msg__FaceBoundingBox __declspec(deprecated)
#endif

namespace servo_service
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct FaceBoundingBox_
{
  using Type = FaceBoundingBox_<ContainerAllocator>;

  explicit FaceBoundingBox_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->xmin = 0.0f;
      this->ymin = 0.0f;
      this->width = 0.0f;
      this->height = 0.0f;
      this->score = 0.0f;
    }
  }

  explicit FaceBoundingBox_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->xmin = 0.0f;
      this->ymin = 0.0f;
      this->width = 0.0f;
      this->height = 0.0f;
      this->score = 0.0f;
    }
  }

  // field types and members
  using _xmin_type =
    float;
  _xmin_type xmin;
  using _ymin_type =
    float;
  _ymin_type ymin;
  using _width_type =
    float;
  _width_type width;
  using _height_type =
    float;
  _height_type height;
  using _score_type =
    float;
  _score_type score;

  // setters for named parameter idiom
  Type & set__xmin(
    const float & _arg)
  {
    this->xmin = _arg;
    return *this;
  }
  Type & set__ymin(
    const float & _arg)
  {
    this->ymin = _arg;
    return *this;
  }
  Type & set__width(
    const float & _arg)
  {
    this->width = _arg;
    return *this;
  }
  Type & set__height(
    const float & _arg)
  {
    this->height = _arg;
    return *this;
  }
  Type & set__score(
    const float & _arg)
  {
    this->score = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    servo_service::msg::FaceBoundingBox_<ContainerAllocator> *;
  using ConstRawPtr =
    const servo_service::msg::FaceBoundingBox_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<servo_service::msg::FaceBoundingBox_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<servo_service::msg::FaceBoundingBox_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      servo_service::msg::FaceBoundingBox_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<servo_service::msg::FaceBoundingBox_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      servo_service::msg::FaceBoundingBox_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<servo_service::msg::FaceBoundingBox_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<servo_service::msg::FaceBoundingBox_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<servo_service::msg::FaceBoundingBox_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__servo_service__msg__FaceBoundingBox
    std::shared_ptr<servo_service::msg::FaceBoundingBox_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__servo_service__msg__FaceBoundingBox
    std::shared_ptr<servo_service::msg::FaceBoundingBox_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const FaceBoundingBox_ & other) const
  {
    if (this->xmin != other.xmin) {
      return false;
    }
    if (this->ymin != other.ymin) {
      return false;
    }
    if (this->width != other.width) {
      return false;
    }
    if (this->height != other.height) {
      return false;
    }
    if (this->score != other.score) {
      return false;
    }
    return true;
  }
  bool operator!=(const FaceBoundingBox_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct FaceBoundingBox_

// alias to use template instance with default allocator
using FaceBoundingBox =
  servo_service::msg::FaceBoundingBox_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace servo_service

#endif  // SERVO_SERVICE__MSG__DETAIL__FACE_BOUNDING_BOX__STRUCT_HPP_
