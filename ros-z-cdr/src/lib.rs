//! CDR (Common Data Representation) serialization for ROS-Z.
//!
//! This crate provides CDR serialization and deserialization compatible with
//! RMW Zenoh for ROS 2 communication.

pub mod buffer;
pub mod deserializer;
pub mod error;
pub mod serializer;
pub mod zbuf_writer;

// Re-export main types for convenience
pub use buffer::CdrBuffer;
// Re-export byteorder types for convenience
pub use byteorder::{BigEndian, LittleEndian};
pub use deserializer::{CdrDeserializer, from_bytes, from_bytes_with};
pub use error::{Error, Result};
pub use serializer::{CdrSerializer, to_buffer, to_vec_fast, to_vec_reuse};
pub use zbuf_writer::ZBufWriter;

/// Native endian type alias for the current platform.
///
/// On little-endian platforms (x86_64, ARM), this is `LittleEndian`.
/// On big-endian platforms, this is `BigEndian`.
///
/// Using `NativeEndian` allows the compiler to optimize away byte-swapping
/// operations when serializing for the native platform.
#[cfg(target_endian = "little")]
pub type NativeEndian = LittleEndian;

#[cfg(target_endian = "big")]
pub type NativeEndian = BigEndian;
