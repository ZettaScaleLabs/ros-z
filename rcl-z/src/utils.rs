use zenoh::Result;
use std::ffi::CStr;
use std::ffi::c_char;


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

pub static NOT_SUPPORTED_CSTR: &CStr = c"NOT_SUPPORTED";

#[macro_export]
macro_rules! rclz_try {
    ($($body:tt)*) => {{
        let x = || -> zenoh::Result<()> {
            $($body)*
            Ok(())
        };
        match x() {
            Ok(_) => RCL_RET_OK as _,
            Err(err) => {
                tracing::error!("{err}");
                RCL_RET_ERROR as _
            }
        }
    }};
}


// pub struct DebugCStr(pub *const c_char);
//
// impl std::fmt::Debug for DebugCStr {
//     fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
//         unsafe {
//             if self.0.is_null() {
//                 write!(f, "null")
//             } else {
//                 match CStr::from_ptr(self.0).to_str() {
//                     Ok(s) => write!(f, "{:?}", s),
//                     Err(_) => write!(f, "invalid utf-8"),
//                 }
//             }
//         }
//     }
// }
