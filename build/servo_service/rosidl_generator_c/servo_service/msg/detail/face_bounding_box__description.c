// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from servo_service:msg/FaceBoundingBox.idl
// generated code does not contain a copyright notice

#include "servo_service/msg/detail/face_bounding_box__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_servo_service
const rosidl_type_hash_t *
servo_service__msg__FaceBoundingBox__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x87, 0x30, 0xf0, 0x44, 0x62, 0x97, 0xd3, 0xd0,
      0x9c, 0x5a, 0x2b, 0x19, 0xaa, 0x59, 0x7e, 0x90,
      0x3f, 0x01, 0xc4, 0x00, 0x78, 0xdd, 0x1f, 0x7a,
      0xfb, 0x23, 0x3a, 0xbc, 0xa8, 0x81, 0x0c, 0x8a,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char servo_service__msg__FaceBoundingBox__TYPE_NAME[] = "servo_service/msg/FaceBoundingBox";

// Define type names, field names, and default values
static char servo_service__msg__FaceBoundingBox__FIELD_NAME__xmin[] = "xmin";
static char servo_service__msg__FaceBoundingBox__FIELD_NAME__ymin[] = "ymin";
static char servo_service__msg__FaceBoundingBox__FIELD_NAME__width[] = "width";
static char servo_service__msg__FaceBoundingBox__FIELD_NAME__height[] = "height";
static char servo_service__msg__FaceBoundingBox__FIELD_NAME__score[] = "score";

static rosidl_runtime_c__type_description__Field servo_service__msg__FaceBoundingBox__FIELDS[] = {
  {
    {servo_service__msg__FaceBoundingBox__FIELD_NAME__xmin, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {servo_service__msg__FaceBoundingBox__FIELD_NAME__ymin, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {servo_service__msg__FaceBoundingBox__FIELD_NAME__width, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {servo_service__msg__FaceBoundingBox__FIELD_NAME__height, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {servo_service__msg__FaceBoundingBox__FIELD_NAME__score, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
servo_service__msg__FaceBoundingBox__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {servo_service__msg__FaceBoundingBox__TYPE_NAME, 33, 33},
      {servo_service__msg__FaceBoundingBox__FIELDS, 5, 5},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "float32 xmin\n"
  "float32 ymin\n"
  "float32 width\n"
  "float32 height\n"
  "float32 score";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
servo_service__msg__FaceBoundingBox__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {servo_service__msg__FaceBoundingBox__TYPE_NAME, 33, 33},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 68, 68},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
servo_service__msg__FaceBoundingBox__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *servo_service__msg__FaceBoundingBox__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
