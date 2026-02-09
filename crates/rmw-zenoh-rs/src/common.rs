use crate::ros::*;

/// Convert RMW return code to RCL return code
///
/// This function is used internally to convert RMW (ROS Middleware) return codes
/// to RCL (ROS Client Library) return codes.
#[unsafe(no_mangle)]
pub extern "C" fn rcl_convert_rmw_ret_to_rcl_ret(rmw_ret: u32) -> rcl_ret_t {
    match rmw_ret {
        RMW_RET_OK => RCL_RET_OK as _,
        RMW_RET_INVALID_ARGUMENT => RCL_RET_INVALID_ARGUMENT as _,
        RMW_RET_BAD_ALLOC => RCL_RET_BAD_ALLOC as _,
        RMW_RET_UNSUPPORTED => RCL_RET_UNSUPPORTED as _,
        // Default: all other RMW errors map to RCL_RET_ERROR
        _ => RCL_RET_ERROR as _,
    }
}
