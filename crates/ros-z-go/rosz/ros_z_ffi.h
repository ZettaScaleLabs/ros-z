#ifndef ROS_Z_FFI_H
#define ROS_Z_FFI_H

#pragma once

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#define ros_z_ZENOH_EVENT_ID_MAX 11

/**
 * Default depth for KEEP_LAST when SYSTEM_DEFAULT (depth=0) is used
 * This matches ROS 2 and rmw_zenoh_cpp behavior
 */
#define ros_z_DEFAULT_HISTORY_DEPTH 10

/**
 * Default shared memory pool size (10 MB).
 */
#define ros_z_DEFAULT_SHM_POOL_SIZE ((10 * 1024) * 1024)

/**
 * Default message size threshold for using SHM (512 bytes).
 *
 * Messages smaller than this will use regular memory allocation.
 * Matches rmw_zenoh_cpp default for compatibility.
 */
#define ros_z_DEFAULT_SHM_THRESHOLD 512

/**
 * Represents a QoS duration in seconds and nanoseconds.
 */
typedef struct ros_z_QosDuration ros_z_QosDuration;

/**
 * Raw publisher for FFI (no type parameters)
 */
typedef struct ros_z_RawPublisher ros_z_RawPublisher;

/**
 * Raw subscriber wrapper that keeps the zenoh subscriber alive
 */
typedef struct ros_z_RawSubscriber ros_z_RawSubscriber;

/**
 * A live ros-z context backed by an open Zenoh session.
 */
typedef struct ros_z_ZContext ros_z_ZContext;

/**
 * A ROS 2-style node.
 */
typedef struct ros_z_ZNode ros_z_ZNode;

/**
 * Opaque node handle for FFI
 */
typedef struct ros_z_node_t {
  struct ros_z_ZNode *inner;
} ros_z_node_t;

/**
 * Opaque context handle for FFI
 */
typedef struct ros_z_context_t {
  struct ros_z_ZContext *inner;
} ros_z_context_t;

/**
 * Configuration struct for context creation
 */
typedef struct ros_z_context_config_t {
  uint32_t domain_id;
  /**
   * Path to a Zenoh JSON5 config file (nullable)
   */
  const char *config_file;
  /**
   * Array of connect endpoint strings (nullable)
   */
  const char *const *connect_endpoints;
  /**
   * Number of connect endpoints
   */
  uintptr_t connect_endpoints_count;
  /**
   * Zenoh mode: "peer", "client", "router" (nullable = default)
   */
  const char *mode;
  /**
   * Whether to disable multicast scouting
   */
  bool disable_multicast_scouting;
  /**
   * Whether to connect to local zenohd on tcp/127.0.0.1:7447
   */
  bool connect_to_local_zenohd;
  /**
   * JSON string for arbitrary Zenoh config overrides (nullable)
   */
  const char *json_config;
  /**
   * Array of remap rule strings in "from:=to" format (nullable)
   */
  const char *const *remap_rules;
  /**
   * Number of remap rules
   */
  uintptr_t remap_rules_count;
  /**
   * Whether to enable logging
   */
  bool enable_logging;
} ros_z_context_config_t;

/**
 * Topic info returned to FFI callers
 */
typedef struct ros_z_topic_info_t {
  char *name;
  char *type_name;
} ros_z_topic_info_t;

/**
 * Node info returned to FFI callers
 */
typedef struct ros_z_node_info_t {
  char *name;
  char *namespace_;
} ros_z_node_info_t;

/**
 * Node configuration for FFI
 */
typedef struct ros_z_node_config_t {
  const char *name;
  const char *namespace_;
  bool enable_type_description_service;
} ros_z_node_config_t;

/**
 * Opaque publisher handle for FFI
 */
typedef struct ros_z_publisher_t {
  struct ros_z_RawPublisher *inner;
} ros_z_publisher_t;

/**
 * C-compatible QoS profile for FFI
 */
typedef struct ros_z_qos_profile_t {
  /**
   * 0 = Reliable (default), 1 = BestEffort
   */
  int32_t reliability;
  /**
   * 0 = Volatile (default), 1 = TransientLocal
   */
  int32_t durability;
  /**
   * 0 = KeepLast (default), 1 = KeepAll
   */
  int32_t history;
  /**
   * Depth for KeepLast (default: 10, ignored for KeepAll)
   */
  int32_t history_depth;
  uint64_t deadline_sec;
  uint64_t deadline_nsec;
  uint64_t lifespan_sec;
  uint64_t lifespan_nsec;
  /**
   * 0 = Automatic (default), 1 = ManualByNode, 2 = ManualByTopic
   */
  int32_t liveliness;
  uint64_t liveliness_lease_sec;
  uint64_t liveliness_lease_nsec;
} ros_z_qos_profile_t;

/**
 * Opaque subscriber handle for FFI
 */
typedef struct ros_z_subscriber_t {
  struct ros_z_RawSubscriber *inner;
} ros_z_subscriber_t;

/**
 * Callback type for receiving messages
 */
typedef void (*ros_z_MessageCallback)(uintptr_t user_data, const uint8_t *data, uintptr_t len);


#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

/**
 * Create a new ros-z context with default config (convenience)
 */
struct ros_z_context_t *ros_z_context_create(uint32_t domain_id);

/**
 * Create a new ros-z context with full configuration
 */
struct ros_z_context_t *ros_z_context_create_with_config(const struct ros_z_context_config_t *config);

/**
 * Shutdown and free context
 */
int32_t ros_z_context_destroy(struct ros_z_context_t *ctx);

/**
 * Get all topic names and types
 */
int32_t ros_z_graph_get_topic_names_and_types(struct ros_z_context_t *ctx,
                                              struct ros_z_topic_info_t **out_topics,
                                              uintptr_t *out_count);

/**
 * Free topic info array
 */
void ros_z_graph_free_topics(struct ros_z_topic_info_t *topics, uintptr_t count);

/**
 * Get all node names and namespaces
 */
int32_t ros_z_graph_get_node_names(struct ros_z_context_t *ctx,
                                   struct ros_z_node_info_t **out_nodes,
                                   uintptr_t *out_count);

/**
 * Free node info array
 */
void ros_z_graph_free_nodes(struct ros_z_node_info_t *nodes, uintptr_t count);

/**
 * Check if a node exists in the graph
 */
int32_t ros_z_graph_node_exists(struct ros_z_context_t *ctx,
                                const char *name,
                                const char *namespace_);

/**
 * Create a new node (simple API)
 */
struct ros_z_node_t *ros_z_node_create(struct ros_z_context_t *ctx,
                                       const char *name,
                                       const char *namespace_);

/**
 * Create a new node with full configuration
 */
struct ros_z_node_t *ros_z_node_create_with_config(struct ros_z_context_t *ctx,
                                                   const struct ros_z_node_config_t *config);

/**
 * Destroy a node
 */
int32_t ros_z_node_destroy(struct ros_z_node_t *node);

/**
 * Create a publisher (default QoS)
 */
struct ros_z_publisher_t *ros_z_publisher_create(struct ros_z_node_t *node,
                                                 const char *topic,
                                                 const char *type_name,
                                                 const char *type_hash);

/**
 * Create a publisher with QoS profile
 */
struct ros_z_publisher_t *ros_z_publisher_create_with_qos(struct ros_z_node_t *node,
                                                          const char *topic,
                                                          const char *type_name,
                                                          const char *type_hash,
                                                          const struct ros_z_qos_profile_t *qos);

/**
 * Publish raw bytes (already CDR serialized)
 */
int32_t ros_z_publisher_publish(struct ros_z_publisher_t *pub_handle,
                                const uint8_t *data,
                                uintptr_t len);

/**
 * Destroy a publisher
 */
int32_t ros_z_publisher_destroy(struct ros_z_publisher_t *pub_handle);

/**
 * Serialize a message to CDR format
 */
int32_t ros_z_serialize(const char *type_name,
                        const uint8_t *msg_data,
                        uintptr_t msg_len,
                        uint8_t **out_ptr,
                        uintptr_t *out_len);

/**
 * Deserialize CDR bytes to raw format for Go
 */
int32_t ros_z_deserialize(const char *type_name,
                          const uint8_t *cdr_data,
                          uintptr_t cdr_len,
                          uint8_t **out_ptr,
                          uintptr_t *out_len);

/**
 * Free bytes allocated by serialize/deserialize
 */
void ros_z_free_bytes(uint8_t *ptr, uintptr_t len);

/**
 * Create a subscriber with callback (default QoS)
 */
struct ros_z_subscriber_t *ros_z_subscriber_create(struct ros_z_node_t *node,
                                                   const char *topic,
                                                   const char *type_name,
                                                   const char *type_hash,
                                                   ros_z_MessageCallback callback,
                                                   uintptr_t user_data);

/**
 * Create a subscriber with callback and QoS profile
 */
struct ros_z_subscriber_t *ros_z_subscriber_create_with_qos(struct ros_z_node_t *node,
                                                            const char *topic,
                                                            const char *type_name,
                                                            const char *type_hash,
                                                            ros_z_MessageCallback callback,
                                                            uintptr_t user_data,
                                                            const struct ros_z_qos_profile_t *qos);

/**
 * Destroy a subscriber
 */
int32_t ros_z_subscriber_destroy(struct ros_z_subscriber_t *sub);

extern void free(void *ptr);

#ifdef __cplusplus
}  // extern "C"
#endif  // __cplusplus

#endif  /* ROS_Z_FFI_H */
