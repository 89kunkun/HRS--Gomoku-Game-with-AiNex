// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from servo_service:msg/FaceBoundingBox.idl
// generated code does not contain a copyright notice
#include "servo_service/msg/detail/face_bounding_box__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
servo_service__msg__FaceBoundingBox__init(servo_service__msg__FaceBoundingBox * msg)
{
  if (!msg) {
    return false;
  }
  // xmin
  // ymin
  // width
  // height
  // score
  return true;
}

void
servo_service__msg__FaceBoundingBox__fini(servo_service__msg__FaceBoundingBox * msg)
{
  if (!msg) {
    return;
  }
  // xmin
  // ymin
  // width
  // height
  // score
}

bool
servo_service__msg__FaceBoundingBox__are_equal(const servo_service__msg__FaceBoundingBox * lhs, const servo_service__msg__FaceBoundingBox * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // xmin
  if (lhs->xmin != rhs->xmin) {
    return false;
  }
  // ymin
  if (lhs->ymin != rhs->ymin) {
    return false;
  }
  // width
  if (lhs->width != rhs->width) {
    return false;
  }
  // height
  if (lhs->height != rhs->height) {
    return false;
  }
  // score
  if (lhs->score != rhs->score) {
    return false;
  }
  return true;
}

bool
servo_service__msg__FaceBoundingBox__copy(
  const servo_service__msg__FaceBoundingBox * input,
  servo_service__msg__FaceBoundingBox * output)
{
  if (!input || !output) {
    return false;
  }
  // xmin
  output->xmin = input->xmin;
  // ymin
  output->ymin = input->ymin;
  // width
  output->width = input->width;
  // height
  output->height = input->height;
  // score
  output->score = input->score;
  return true;
}

servo_service__msg__FaceBoundingBox *
servo_service__msg__FaceBoundingBox__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  servo_service__msg__FaceBoundingBox * msg = (servo_service__msg__FaceBoundingBox *)allocator.allocate(sizeof(servo_service__msg__FaceBoundingBox), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(servo_service__msg__FaceBoundingBox));
  bool success = servo_service__msg__FaceBoundingBox__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
servo_service__msg__FaceBoundingBox__destroy(servo_service__msg__FaceBoundingBox * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    servo_service__msg__FaceBoundingBox__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
servo_service__msg__FaceBoundingBox__Sequence__init(servo_service__msg__FaceBoundingBox__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  servo_service__msg__FaceBoundingBox * data = NULL;

  if (size) {
    data = (servo_service__msg__FaceBoundingBox *)allocator.zero_allocate(size, sizeof(servo_service__msg__FaceBoundingBox), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = servo_service__msg__FaceBoundingBox__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        servo_service__msg__FaceBoundingBox__fini(&data[i - 1]);
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
servo_service__msg__FaceBoundingBox__Sequence__fini(servo_service__msg__FaceBoundingBox__Sequence * array)
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
      servo_service__msg__FaceBoundingBox__fini(&array->data[i]);
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

servo_service__msg__FaceBoundingBox__Sequence *
servo_service__msg__FaceBoundingBox__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  servo_service__msg__FaceBoundingBox__Sequence * array = (servo_service__msg__FaceBoundingBox__Sequence *)allocator.allocate(sizeof(servo_service__msg__FaceBoundingBox__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = servo_service__msg__FaceBoundingBox__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
servo_service__msg__FaceBoundingBox__Sequence__destroy(servo_service__msg__FaceBoundingBox__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    servo_service__msg__FaceBoundingBox__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
servo_service__msg__FaceBoundingBox__Sequence__are_equal(const servo_service__msg__FaceBoundingBox__Sequence * lhs, const servo_service__msg__FaceBoundingBox__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!servo_service__msg__FaceBoundingBox__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
servo_service__msg__FaceBoundingBox__Sequence__copy(
  const servo_service__msg__FaceBoundingBox__Sequence * input,
  servo_service__msg__FaceBoundingBox__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(servo_service__msg__FaceBoundingBox);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    servo_service__msg__FaceBoundingBox * data =
      (servo_service__msg__FaceBoundingBox *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!servo_service__msg__FaceBoundingBox__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          servo_service__msg__FaceBoundingBox__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!servo_service__msg__FaceBoundingBox__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
