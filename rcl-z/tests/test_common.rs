#![cfg(feature = "test-core")]

use rcl_z::{
    common::rcl_convert_rmw_ret_to_rcl_ret,
    ros::{
        RCL_RET_BAD_ALLOC, RCL_RET_ERROR, RCL_RET_INVALID_ARGUMENT, RCL_RET_NODE_NAME_NON_EXISTENT,
        RCL_RET_OK, RCL_RET_UNSUPPORTED, RMW_RET_BAD_ALLOC, RMW_RET_ERROR,
        RMW_RET_INCORRECT_RMW_IMPLEMENTATION, RMW_RET_INVALID_ARGUMENT,
        RMW_RET_NODE_NAME_NON_EXISTENT, RMW_RET_OK, RMW_RET_TIMEOUT, RMW_RET_UNSUPPORTED,
    },
};

/// Test rcl_convert_rmw_ret_to_rcl_ret function
/// Aligns with test_common.cpp::test_rmw_ret_to_rcl_ret
#[test]
fn test_rmw_ret_to_rcl_ret() {
    // Test specific mappings
    assert_eq!(
        rcl_convert_rmw_ret_to_rcl_ret(RMW_RET_OK),
        RCL_RET_OK as i32
    );
    assert_eq!(
        rcl_convert_rmw_ret_to_rcl_ret(RMW_RET_INVALID_ARGUMENT),
        RCL_RET_INVALID_ARGUMENT as i32
    );
    assert_eq!(
        rcl_convert_rmw_ret_to_rcl_ret(RMW_RET_BAD_ALLOC),
        RCL_RET_BAD_ALLOC as i32
    );
    assert_eq!(
        rcl_convert_rmw_ret_to_rcl_ret(RMW_RET_UNSUPPORTED),
        RCL_RET_UNSUPPORTED as i32
    );
    assert_eq!(
        rcl_convert_rmw_ret_to_rcl_ret(RMW_RET_NODE_NAME_NON_EXISTENT),
        RCL_RET_NODE_NAME_NON_EXISTENT as i32
    );

    // Test default behavior - all unmapped errors should return RCL_RET_ERROR
    assert_eq!(
        rcl_convert_rmw_ret_to_rcl_ret(RMW_RET_ERROR),
        RCL_RET_ERROR as i32
    );
    assert_eq!(
        rcl_convert_rmw_ret_to_rcl_ret(RMW_RET_TIMEOUT),
        RCL_RET_ERROR as i32
    );
    assert_eq!(
        rcl_convert_rmw_ret_to_rcl_ret(RMW_RET_INCORRECT_RMW_IMPLEMENTATION),
        RCL_RET_ERROR as i32
    );
}
