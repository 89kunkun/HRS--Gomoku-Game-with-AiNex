// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from servo_service:msg/FaceBoundingBoxArray.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "servo_service/msg/face_bounding_box_array.hpp"


#ifndef SERVO_SERVICE__MSG__DETAIL__FACE_BOUNDING_BOX_ARRAY__STRUCT_HPP_
#define SERVO_SERVICE__MSG__DETAIL__FACE_BOUNDING_BOX_ARRAY__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"
// Member 'boxes'
#include "servo_service/msg/detail/face_bounding_box__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__servo_service__msg__FaceBoundingBoxArray __attribute__((deprecated))
#else
# define DEPRECATED__servo_service__msg__FaceBoundingBoxArray __declspec(deprecated)
#endif

namespace servo_service
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct FaceBoundingBoxArray_
{
  using Type = FaceBoundingBoxArray_<ContainerAllocator>;

  explicit FaceBoundingBoxArray_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    (void)_init;
  }

  explicit FaceBoundingBoxArray_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _boxes_type =
    std::vector<servo_service::msg::FaceBoundingBox_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<servo_service::msg::FaceBoundingBox_<ContainerAllocator>>>;
  _boxes_type boxes;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__boxes(
    const std::vector<servo_service::msg::FaceBoundingBox_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<servo_service::msg::FaceBoundingBox_<ContainerAllocator>>> & _arg)
  {
    this->boxes = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    servo_service::msg::FaceBoundingBoxArray_<ContainerAllocator> *;
  using ConstRawPtr =
    const servo_service::msg::FaceBoundingBoxArray_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<servo_service::msg::FaceBoundingBoxArray_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<servo_service::msg::FaceBoundingBoxArray_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      servo_service::msg::FaceBoundingBoxArray_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<servo_service::msg::FaceBoundingBoxArray_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      servo_service::msg::FaceBoundingBoxArray_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<servo_service::msg::FaceBoundingBoxArray_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<servo_service::msg::FaceBoundingBoxArray_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<servo_service::msg::FaceBoundingBoxArray_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__servo_service__msg__FaceBoundingBoxArray
    std::shared_ptr<servo_service::msg::FaceBoundingBoxArray_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__servo_service__msg__FaceBoundingBoxArray
    std::shared_ptr<servo_service::msg::FaceBoundingBoxArray_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const FaceBoundingBoxArray_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->boxes != other.boxes) {
      return false;
    }
    return true;
  }
  bool operator!=(const FaceBoundingBoxArray_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct FaceBoundingBoxArray_

// alias to use template instance with default allocator
using FaceBoundingBoxArray =
  servo_service::msg::FaceBoundingBoxArray_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace servo_service

#endif  // SERVO_SERVICE__MSG__DETAIL__FACE_BOUNDING_BOX_ARRAY__STRUCT_HPP_
