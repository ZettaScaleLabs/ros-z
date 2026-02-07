use std::ffi::CStr;
use std::ffi::c_char;
use zenoh::Result;

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

#[derive(Debug, Default)]
pub struct Notifier {
    pub(crate) mutex: parking_lot::Mutex<()>,
    pub(crate) cv: parking_lot::Condvar,
}

impl Notifier {
    pub fn notify_all(&self) {
        // No need to lock mutex for notify_all with parking_lot
        // Calling notify_all() without holding the lock avoids thundering herd issues
        self.cv.notify_all();
    }
}

#[macro_export]
macro_rules! impl_has_impl_ptr {
    ($ctype:ty, $cimpl_type:ty, $impl_type:ty) => {
        impl $crate::traits::HasImplPtr for $ctype {
            type ImplType = $impl_type;
            type CImplType = $cimpl_type;
            fn get_impl(&self) -> *mut Self::CImplType {
                self.impl_
            }
            fn get_mut_impl(&mut self) -> &mut *mut Self::CImplType {
                &mut self.impl_
            }
        }
    };
}

pub fn parse_args(argc: i32, argv: *const *const c_char) -> Vec<String> {
    let mut args = Vec::new();

    unsafe {
        for i in 0..argc {
            // Get the pointer to the i-th C string
            let c_str_ptr = *argv.offset(i as isize);
            if c_str_ptr.is_null() {
                continue;
            }

            // Convert to &CStr then to String
            let c_str = CStr::from_ptr(c_str_ptr);
            let str_slice = c_str.to_string_lossy().into_owned();
            args.push(str_slice);
        }
    }

    args
}
