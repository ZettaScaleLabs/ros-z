#include "serde_bridge.h"
#include <fastcdr/Cdr.h>
#include <fastcdr/FastBuffer.h>
#include <rosidl_typesupport_fastrtps_c/identifier.h>
#include <rosidl_typesupport_fastrtps_cpp/identifier.hpp>
#include <rosidl_typesupport_fastrtps_cpp/message_type_support.h>
#include <rosidl_typesupport_interface/macros.h>

const rosidl_message_type_support_t *
serde_bridge::get_message_typesupport(const rosidl_message_type_support_t *ts) {
  if (!ts) {
    return nullptr;
  }
  auto type_support = get_message_typesupport_handle(
      ts, rosidl_typesupport_fastrtps_c__identifier);
  if (!type_support) {
    type_support = get_message_typesupport_handle(
        ts, rosidl_typesupport_fastrtps_cpp::typesupport_identifier);
  }

  if (!type_support) {
    return nullptr;
  }
  return type_support;
}

size_t
serde_bridge::get_serialized_size(const rosidl_message_type_support_t *ts,
                                  const void *ros_message) {
  const message_type_support_callbacks_t *callbacks =
      static_cast<const message_type_support_callbacks_t *>(ts->data);
  return 4 + callbacks->get_serialized_size(ros_message);
}

bool serde_bridge::serialize_message(const rosidl_message_type_support_t *ts,
                                     const void *ros_message,
                                     rust::Vec<uint8_t> &out) {
  if (!ts || !ros_message) {
    return false;
  }

  // TODO: type hash
  // const rosidl_type_hash_t * type_hash =
  // type_support->get_type_hash_func(type_support);
  auto callbacks =
      static_cast<const message_type_support_callbacks_t *>(ts->data);

  eprosima::fastcdr::FastBuffer buffer(reinterpret_cast<char *>(out.data()),
                                       out.size());
  eprosima::fastcdr::Cdr ser(buffer, eprosima::fastcdr::Cdr::DEFAULT_ENDIAN,
                             eprosima::fastcdr::CdrVersion::DDS_CDR);

  ser.serialize_encapsulation();
  return callbacks->cdr_serialize(ros_message, ser);
}

bool serde_bridge::deserialize_message(const rosidl_message_type_support_t *ts,
                                       const rust::Vec<uint8_t> &data,
                                       void *ros_message) {
  if (!ts || !ros_message) {
    return false;
  }

  auto callbacks =
      static_cast<const message_type_support_callbacks_t *>(ts->data);

  eprosima::fastcdr::FastBuffer buffer(
      reinterpret_cast<char *>(const_cast<uint8_t *>(data.data())),
      data.size());
  eprosima::fastcdr::Cdr cdr(buffer);
  cdr.read_encapsulation();

  return callbacks->cdr_deserialize(cdr, ros_message);
}
