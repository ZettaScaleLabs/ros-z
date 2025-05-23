#![allow(unused)]

use crate::ros::*;

macro_rules! impl_trivial_ok_fn {
    (
        pub fn $fn_name:ident (
            $( $arg_name:ident : $arg_ty:ty ),* $(,)?
        ) -> $ret:ty ;
    ) => {
        #[unsafe(no_mangle)]
        pub extern "C" fn $fn_name(
            $( $arg_name : $arg_ty ),*
        ) -> $ret {
            tracing::trace!(stringify!($fn_name));
            RCL_RET_OK as _
        }
    };
}

impl_trivial_ok_fn! {
    pub fn rcl_take_request_with_info(
        service: *const rcl_service_t,
        request_header: *mut rmw_service_info_t,
        ros_request: *mut ::std::os::raw::c_void,
    ) -> rcl_ret_t;
}
