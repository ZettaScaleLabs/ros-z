#pragma once

#include "rust/cxx.h"
#include <rosidl_typesupport_fastrtps_cpp/message_type_support.h>

using c_void = void;

namespace serde_bridge {

bool serialize_message(const rosidl_message_type_support_t *ts,
                       const void *ros_message, rust::Vec<uint8_t> &out);

bool deserialize_message(const rosidl_message_type_support_t *ts,
                         const rust::Vec<uint8_t> &data, void *ros_message);

const rosidl_message_type_support_t *
get_message_typesupport(const rosidl_message_type_support_t *ts);

size_t get_serialized_size(const rosidl_message_type_support_t *ts,
                           const void *ros_message);

}; // namespace serde_bridge
