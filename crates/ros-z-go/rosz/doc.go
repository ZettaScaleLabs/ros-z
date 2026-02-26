// Package rosz provides Go bindings for ros-z, a Zenoh-native ROS 2 implementation.
//
// The package supports pub/sub with automatic CDR serialization,
// builder-pattern resource creation, and idempotent cleanup.
//
// All resource types (Context, Node, Publisher, Subscriber) must be closed
// after use. Each type's Close() method is idempotent and safe to call
// multiple times.
//
// Callbacks registered with BuildWithCallback are invoked on C/Rust threads.
// Avoid long blocking operations in callbacks to prevent stalling the Zenoh runtime.
package rosz
