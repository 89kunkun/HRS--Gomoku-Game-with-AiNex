// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from servo_service:msg/FaceBoundingBoxArray.idl
// generated code does not contain a copyright notice
#include "servo_service/msg/detail/face_bounding_box_array__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `boxes`
#include "servo_service/msg/detail/face_bounding_box__functions.h"

bool
servo_service__msg__FaceBoundingBoxArray__init(servo_service__msg__FaceBoundingBoxArray * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    servo_service__msg__FaceBoundingBoxArray__fini(msg);
    return false;
  }
  // boxes
  if (!servo_service__msg__FaceBoundingBox__Sequence__init(&msg->boxes, 0)) {
    servo_service__msg__FaceBoundingBoxArray__fini(msg);
    return false;
  }
  return true;
}

void
servo_service__msg__FaceBoundingBoxArray__fini(servo_service__msg__FaceBoundingBoxArray * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // boxes
  servo_service__msg__FaceBoundingBox__Sequence__fini(&msg->boxes);
}

bool
servo_service__msg__FaceBoundingBoxArray__are_equal(const servo_service__msg__FaceBoundingBoxArray * lhs, const servo_service__msg__FaceBoundingBoxArray * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // boxes
  if (!servo_service__msg__FaceBoundingBox__Sequence__are_equal(
      &(lhs->boxes), &(rhs->boxes)))
  {
    return false;
  }
  return true;
}

bool
servo_service__msg__FaceBoundingBoxArray__copy(
  const servo_service__msg__FaceBoundingBoxArray * input,
  servo_service__msg__FaceBoundingBoxArray * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // boxes
  if (!servo_service__msg__FaceBoundingBox__Sequence__copy(
      &(input->boxes), &(output->boxes)))
  {
    return false;
  }
  return true;
}

servo_service__msg__FaceBoundingBoxArray *
servo_service__msg__FaceBoundingBoxArray__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  servo_service__msg__FaceBoundingBoxArray * msg = (servo_service__msg__FaceBoundingBoxArray *)allocator.allocate(sizeof(servo_service__msg__FaceBoundingBoxArray), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(servo_service__msg__FaceBoundingBoxArray));
  bool success = servo_service__msg__FaceBoundingBoxArray__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
servo_service__msg__FaceBoundingBoxArray__destroy(servo_service__msg__FaceBoundingBoxArray * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    servo_service__msg__FaceBoundingBoxArray__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
servo_service__msg__FaceBoundingBoxArray__Sequence__init(servo_service__msg__FaceBoundingBoxArray__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  servo_service__msg__FaceBoundingBoxArray * data = NULL;

  if (size) {
    data = (servo_service__msg__FaceBoundingBoxArray *)allocator.zero_allocate(size, sizeof(servo_service__msg__FaceBoundingBoxArray), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = servo_service__msg__FaceBoundingBoxArray__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        servo_service__msg__FaceBoundingBoxArray__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
servo_service__msg__FaceBoundingBoxArray__Sequence__fini(servo_service__msg__FaceBoundingBoxArray__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      servo_service__msg__FaceBoundingBoxArray__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

servo_service__msg__FaceBoundingBoxArray__Sequence *
servo_service__msg__FaceBoundingBoxArray__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  servo_service__msg__FaceBoundingBoxArray__Sequence * array = (servo_service__msg__FaceBoundingBoxArray__Sequence *)allocator.allocate(sizeof(servo_service__msg__FaceBoundingBoxArray__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = servo_service__msg__FaceBoundingBoxArray__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
servo_service__msg__FaceBoundingBoxArray__Sequence__destroy(servo_service__msg__FaceBoundingBoxArray__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    servo_service__msg__FaceBoundingBoxArray__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
servo_service__msg__FaceBoundingBoxArray__Sequence__are_equal(const servo_service__msg__FaceBoundingBoxArray__Sequence * lhs, const servo_service__msg__FaceBoundingBoxArray__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!servo_service__msg__FaceBoundingBoxArray__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
servo_service__msg__FaceBoundingBoxArray__Sequence__copy(
  const servo_service__msg__FaceBoundingBoxArray__Sequence * input,
  servo_service__msg__FaceBoundingBoxArray__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(servo_service__msg__FaceBoundingBoxArray);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    servo_service__msg__FaceBoundingBoxArray * data =
      (servo_service__msg__FaceBoundingBoxArray *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!servo_service__msg__FaceBoundingBoxArray__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          servo_service__msg__FaceBoundingBoxArray__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!servo_service__msg__FaceBoundingBoxArray__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
