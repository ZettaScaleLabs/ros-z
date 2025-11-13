#![allow(clippy::enum_variant_names)]
#![allow(clippy::missing_safety_doc)]
#![allow(clippy::explicit_auto_deref)]
#![allow(clippy::collapsible_if)]
#![allow(clippy::cast_slice_from_raw_parts)]
#![allow(clippy::not_unsafe_ptr_arg_deref)]
#![allow(unpredictable_function_pointer_comparisons)]

mod qos;
mod ros;
mod traits;
mod utils;

pub mod arguments;
pub mod context;
pub mod graph;
pub mod guard_condition;
pub mod init;
pub mod msg;
pub mod node;
pub mod pubsub;
pub mod service;
pub mod timer;
pub mod type_support;
pub mod wait_set;

/// Newtype wrapper for a C void. Only useful as a `*c_void`
#[allow(non_camel_case_types)]
#[repr(transparent)]
pub struct c_void(pub ::std::os::raw::c_void);

/// # Safety
///
/// We assert that the namespace and type ID refer to a C++
/// type which is equivalent to this Rust type.
unsafe impl cxx::ExternType for c_void {
    type Id = cxx::type_id!(c_void);
    type Kind = cxx::kind::Trivial;
}
