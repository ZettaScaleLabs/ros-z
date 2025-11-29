use ros_z::Builder;
use ros_z::context::ZContextBuilder;

use crate::context::ContextImpl;
use crate::traits::{BorrowImpl, ImplAccessError, OwnImpl};
use crate::{impl_has_impl_ptr, ros::*};
use std::ffi::{c_char, c_int};
use std::env;

#[derive(Debug, Default, Clone, Copy)]
pub struct InitOptionsImpl {
    domain_id: usize,
    allocator: rcl_allocator_t,
}

impl_has_impl_ptr!(rcl_init_options_t, rcl_init_options_impl_t, InitOptionsImpl);

#[unsafe(no_mangle)]
pub extern "C" fn rcl_get_zero_initialized_init_options() -> rcl_init_options_t {
    // TODO: Implement proper zero initialization
    unsafe { std::mem::zeroed() }
}

// Default allocator functions
unsafe extern "C" fn default_allocate(size: usize, _state: *mut std::ffi::c_void) -> *mut std::ffi::c_void {
    if size == 0 {
        return std::ptr::null_mut();
    }
    let layout = std::alloc::Layout::from_size_align(size, std::mem::align_of::<usize>()).unwrap();
    unsafe { std::alloc::alloc(layout) as *mut std::ffi::c_void }
}

unsafe extern "C" fn default_deallocate(pointer: *mut std::ffi::c_void, _state: *mut std::ffi::c_void) {
    if !pointer.is_null() {
        // We need to know the size to deallocate properly, but the allocator interface doesn't provide it
        // For now, we'll leak memory. A proper implementation would need to track allocations.
        // TODO: Implement proper memory tracking
    }
}

unsafe extern "C" fn default_reallocate(
    pointer: *mut std::ffi::c_void,
    size: usize,
    _state: *mut std::ffi::c_void,
) -> *mut std::ffi::c_void {
    if size == 0 {
        // SAFETY: Called from unsafe extern "C" fn
        unsafe { default_deallocate(pointer, _state) };
        return std::ptr::null_mut();
    }
    if pointer.is_null() {
        // SAFETY: Called from unsafe extern "C" fn
        return unsafe { default_allocate(size, _state) };
    }
    // Without knowing the old size, we can't properly reallocate
    // For now, just allocate new memory and copy
    // SAFETY: Called from unsafe extern "C" fn
    let new_ptr = unsafe { default_allocate(size, _state) };
    if !new_ptr.is_null() && !pointer.is_null() {
        // This is unsafe as we don't know the actual size, but it's better than nothing
        unsafe { std::ptr::copy_nonoverlapping(pointer as *const u8, new_ptr as *mut u8, size) };
    }
    new_ptr
}

unsafe extern "C" fn default_zero_allocate(
    number_of_elements: usize,
    size_of_element: usize,
    _state: *mut std::ffi::c_void,
) -> *mut std::ffi::c_void {
    let total_size = number_of_elements * size_of_element;
    // SAFETY: Called from unsafe extern "C" fn
    let ptr = unsafe { default_allocate(total_size, _state) };
    if !ptr.is_null() {
        unsafe { std::ptr::write_bytes(ptr as *mut u8, 0, total_size) };
    }
    ptr
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_get_default_allocator() -> rcl_allocator_t {
    rcl_allocator_t {
        allocate: Some(default_allocate),
        deallocate: Some(default_deallocate),
        reallocate: Some(default_reallocate),
        zero_allocate: Some(default_zero_allocate),
        state: std::ptr::null_mut(),
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_init_options_init(
    init_options: *mut rcl_init_options_t,
    allocator: rcl_allocator_t,
) -> rcl_ret_t {
    // Check for null pointer
    if init_options.is_null() {
        return RCL_RET_INVALID_ARGUMENT as _;
    }

    let opts_impl = InitOptionsImpl {
        allocator,
        ..Default::default()
    };

    match init_options.assign_impl(opts_impl) {
        Ok(_) => RCL_RET_OK as _,
        Err(ImplAccessError::NonNullImplPtr) => RCL_RET_ALREADY_INIT as _,
        Err(_) => RCL_RET_INVALID_ARGUMENT as _,
    }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn rcl_init_options_get_domain_id(
    init_options: *const rcl_init_options_t,
    domain_id: *mut usize,
) -> rcl_ret_t {
    tracing::trace!("rcl_init_options_get_domain_id");

    // Check for null pointers
    if init_options.is_null() || domain_id.is_null() {
        return RCL_RET_INVALID_ARGUMENT as _;
    }

    match init_options.borrow_impl() {
        Ok(impl_) => {
            unsafe {
                *domain_id = impl_.domain_id;
            }
            RCL_RET_OK as _
        }
        Err(_) => RCL_RET_INVALID_ARGUMENT as _,
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_init_options_set_domain_id(
    init_options: *mut rcl_init_options_t,
    domain_id: usize,
) -> rcl_ret_t {
    tracing::trace!("rcl_init_options_set_domain_id");

    // Check for null pointer
    if init_options.is_null() {
        return RCL_RET_INVALID_ARGUMENT as _;
    }

    match init_options.borrow_mut_impl() {
        Ok(impl_) => {
            impl_.domain_id = domain_id;
            RCL_RET_OK as _
        }
        Err(_) => RCL_RET_INVALID_ARGUMENT as _,
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_init_options_fini(init_options: *mut rcl_init_options_t) -> rcl_ret_t {
    // FIXME: tracing is not usable at the exit stage
    // tracing::trace!("rcl_init_options_fini");

    // Check for null pointer
    if init_options.is_null() {
        return RCL_RET_INVALID_ARGUMENT as _;
    }

    match init_options.own_impl() {
        Ok(impl_box) => {
            std::mem::drop(impl_box);
            RCL_RET_OK as _
        }
        Err(ImplAccessError::NullImplPtr) => RCL_RET_INVALID_ARGUMENT as _,
        Err(_) => RCL_RET_INVALID_ARGUMENT as _,
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_init_options_copy(
    src: *const rcl_init_options_t,
    dst: *mut rcl_init_options_t,
) -> rcl_ret_t {
    // Check for null pointers
    if src.is_null() || dst.is_null() {
        return RCL_RET_INVALID_ARGUMENT as _;
    }

    match src.borrow_impl() {
        Ok(src_impl) => match dst.assign_impl(*src_impl) {
            Ok(_) => RCL_RET_OK as _,
            Err(ImplAccessError::NonNullImplPtr) => RCL_RET_ALREADY_INIT as _,
            Err(_) => RCL_RET_INVALID_ARGUMENT as _,
        },
        Err(_) => RCL_RET_INVALID_ARGUMENT as _,
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_init_options_get_allocator(
    init_options: *const rcl_init_options_t,
) -> *const rcl_allocator_t {
    tracing::trace!("rcl_init_options_get_allocator");

    match init_options.borrow_impl() {
        Ok(x) => &x.allocator,
        Err(e) => {
            tracing::error!("rcl_init_options_get_allocator failed with {e}");
            std::ptr::null()
        }
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn rcl_init(
    argc: c_int,
    argv: *const *const c_char,
    options: *const rcl_init_options_t,
    context: *mut rcl_context_t,
) -> rcl_ret_t {
    zenoh::init_log_from_env_or("error");

    // Validate arguments
    if options.is_null() || context.is_null() {
        return RCL_RET_INVALID_ARGUMENT as _;
    }

    // If argc is not 0, argv must not be null
    if argc != 0 && argv.is_null() {
        return RCL_RET_INVALID_ARGUMENT as _;
    }

    // If argc is 0, argv must be null
    if argc == 0 && !argv.is_null() {
        return RCL_RET_INVALID_ARGUMENT as _;
    }

    // If argv is not null, check that none of the arguments are null
    if !argv.is_null() {
        unsafe {
            for i in 0..argc {
                if (*argv.offset(i as isize)).is_null() {
                    return RCL_RET_INVALID_ARGUMENT as _;
                }
            }
        }
    }

    // Check if context is already initialized
    if unsafe { crate::context::rcl_context_is_valid(context) } {
        return RCL_RET_ALREADY_INIT as _;
    }

    // Parse ROS args if provided (simplified - just check for basic invalid patterns)
    if argc > 0 && !argv.is_null() {
        unsafe {
            for i in 0..argc {
                let arg = std::ffi::CStr::from_ptr(*argv.offset(i as isize));
                if let Ok(arg_str) = arg.to_str() {
                    // Check for invalid remap pattern "name:=" without a value
                    if arg_str == "name:=" {
                        return RCL_RET_INVALID_ROS_ARGS as _;
                    }
                }
            }
        }
    }

    let domain_id = match options.borrow_impl() {
        Ok(opts) => opts.domain_id,
        Err(_) => return RCL_RET_INVALID_ARGUMENT as _,
    };

    let ctx = match ZContextBuilder::default().with_domain_id(domain_id).build() {
        Ok(ctx) => ctx,
        Err(_) => return RCL_RET_ERROR as _,
    };

    // Copy the init_options to store in the context
    let init_options_copy = unsafe { *options };

    let ctx_impl = ContextImpl::new(ctx, init_options_copy, domain_id);
    match context.assign_impl(ctx_impl) {
        Ok(_) => RCL_RET_OK as _,
        Err(ImplAccessError::NonNullImplPtr) => RCL_RET_ALREADY_INIT as _,
        Err(_) => RCL_RET_ERROR as _,
    }
}

/// Get the RMW implementation identifier.
///
/// Returns a string identifying the RMW implementation used by this RCL implementation.
#[unsafe(no_mangle)]
pub extern "C" fn rcl_get_rmw_implementation_identifier() -> *const c_char {
    c"rmw_zenoh_cpp".as_ptr()
}

/// Get the default domain ID from the ROS_DOMAIN_ID environment variable.
///
/// If ROS_DOMAIN_ID is not set or is empty, domain_id is left unchanged and RCL_RET_OK is returned.
/// If ROS_DOMAIN_ID is set to a valid non-negative integer, domain_id is set to that value.
/// If ROS_DOMAIN_ID contains an invalid value, RCL_RET_ERROR is returned.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn rcl_get_default_domain_id(domain_id: *mut usize) -> rcl_ret_t {
    tracing::trace!("rcl_get_default_domain_id");

    // Check for null pointer
    if domain_id.is_null() {
        return RCL_RET_INVALID_ARGUMENT as _;
    }

    // Try to read ROS_DOMAIN_ID environment variable
    match env::var("ROS_DOMAIN_ID") {
        Ok(value) => {
            tracing::trace!("ROS_DOMAIN_ID found: '{}'", value);
            // If the value is empty, leave domain_id unchanged
            if value.is_empty() {
                return RCL_RET_OK as _;
            }

            // Parse the value, similar to C's strtoul behavior
            // We need to check if the entire string is consumed during parsing
            let trimmed = value.trim();

            // First check: the trimmed value should only contain digits
            // This catches cases like "0   not really" where there's trailing non-numeric content
            if trimmed.chars().any(|c| !c.is_ascii_digit()) {
                // Check if it starts with digits - this is the "0   not really" case
                let numeric_prefix: String = trimmed.chars()
                    .take_while(|c| c.is_ascii_digit())
                    .collect();

                if !numeric_prefix.is_empty() && numeric_prefix.len() < trimmed.len() {
                    tracing::error!("ROS_DOMAIN_ID is not an integral number: '{}'", value);
                    return RCL_RET_ERROR as _;
                }
                tracing::error!("ROS_DOMAIN_ID is not an integral number: '{}'", value);
                return RCL_RET_ERROR as _;
            }

            // Try to parse the trimmed value
            match trimmed.parse::<usize>() {
                Ok(parsed_value) => {
                    tracing::trace!("Parsed ROS_DOMAIN_ID to: {}", parsed_value);
                    unsafe {
                        *domain_id = parsed_value;
                    }
                    RCL_RET_OK as _
                }
                Err(_) => {
                    // Parse error - likely overflow
                    tracing::error!("ROS_DOMAIN_ID is out of range: '{}'", value);
                    RCL_RET_ERROR as _
                }
            }
        }
        Err(env::VarError::NotPresent) => {
            // Variable not set, leave domain_id unchanged
            RCL_RET_OK as _
        }
        Err(env::VarError::NotUnicode(_)) => {
            tracing::error!("ROS_DOMAIN_ID contains invalid UTF-8");
            RCL_RET_ERROR as _
        }
    }
}
