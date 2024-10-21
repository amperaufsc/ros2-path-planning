// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from fs_msgs:msg/GoSignal.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "fs_msgs/msg/detail/go_signal__rosidl_typesupport_introspection_c.h"
#include "fs_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "fs_msgs/msg/detail/go_signal__functions.h"
#include "fs_msgs/msg/detail/go_signal__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `mission`
// Member `track`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void fs_msgs__msg__GoSignal__rosidl_typesupport_introspection_c__GoSignal_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  fs_msgs__msg__GoSignal__init(message_memory);
}

void fs_msgs__msg__GoSignal__rosidl_typesupport_introspection_c__GoSignal_fini_function(void * message_memory)
{
  fs_msgs__msg__GoSignal__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember fs_msgs__msg__GoSignal__rosidl_typesupport_introspection_c__GoSignal_message_member_array[3] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(fs_msgs__msg__GoSignal, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "mission",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(fs_msgs__msg__GoSignal, mission),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "track",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(fs_msgs__msg__GoSignal, track),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers fs_msgs__msg__GoSignal__rosidl_typesupport_introspection_c__GoSignal_message_members = {
  "fs_msgs__msg",  // message namespace
  "GoSignal",  // message name
  3,  // number of fields
  sizeof(fs_msgs__msg__GoSignal),
  fs_msgs__msg__GoSignal__rosidl_typesupport_introspection_c__GoSignal_message_member_array,  // message members
  fs_msgs__msg__GoSignal__rosidl_typesupport_introspection_c__GoSignal_init_function,  // function to initialize message memory (memory has to be allocated)
  fs_msgs__msg__GoSignal__rosidl_typesupport_introspection_c__GoSignal_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t fs_msgs__msg__GoSignal__rosidl_typesupport_introspection_c__GoSignal_message_type_support_handle = {
  0,
  &fs_msgs__msg__GoSignal__rosidl_typesupport_introspection_c__GoSignal_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_fs_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, fs_msgs, msg, GoSignal)() {
  fs_msgs__msg__GoSignal__rosidl_typesupport_introspection_c__GoSignal_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  if (!fs_msgs__msg__GoSignal__rosidl_typesupport_introspection_c__GoSignal_message_type_support_handle.typesupport_identifier) {
    fs_msgs__msg__GoSignal__rosidl_typesupport_introspection_c__GoSignal_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &fs_msgs__msg__GoSignal__rosidl_typesupport_introspection_c__GoSignal_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
