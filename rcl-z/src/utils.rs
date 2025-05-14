use zenoh::Result;
use std::ffi::CStr;
use std::ffi::c_char;
use crate::ros::rcl_serialized_message_t;


/// Safely converts a C string pointer to a Rust `&str`.
///
/// # Safety
/// - `ptr` must be non-null.
/// - Must point to a valid null-terminated UTF-8 C string.
pub fn str_from_ptr<'a>(ptr: *const c_char) -> Result<&'a str> {
    if ptr.is_null() {
        return Err("Received null pointer for C string".into());
    }

    let cstr = unsafe { CStr::from_ptr(ptr) };
    Ok(cstr.to_str()?)
}
