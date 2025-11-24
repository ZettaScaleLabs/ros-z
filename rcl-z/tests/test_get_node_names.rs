#![cfg(feature = "test-core")]

use std::{collections::HashSet, ffi::CStr, ptr, thread, time::Duration};

use rcl_z::{
    context::{rcl_context_fini, rcl_get_zero_initialized_context, rcl_shutdown},
    graph::{
        rcl_get_node_names, rcutils_get_zero_initialized_string_array, rcutils_string_array_fini,
    },
    init::{
        rcl_get_default_allocator, rcl_get_zero_initialized_init_options, rcl_init,
        rcl_init_options_fini, rcl_init_options_init, rcl_init_options_set_domain_id,
    },
    node::{
        rcl_get_zero_initialized_node, rcl_node_fini, rcl_node_get_default_options, rcl_node_init,
    },
    ros::*,
};

/// Test fixture that provides an initialized RCL context
struct TestGetNodeNamesFixture {
    context: rcl_context_t,
    init_options: rcl_init_options_t,
}

impl TestGetNodeNamesFixture {
    fn new(domain_id: usize) -> Self {
        // Initialize context with unique domain_id to isolate from other tests
        let mut init_options = rcl_get_zero_initialized_init_options();
        let ret = rcl_init_options_init(&mut init_options, rcl_get_default_allocator());
        assert_eq!(ret, RCL_RET_OK as i32, "Failed to initialize init options");

        // Use specified domain_id to isolate this test from others
        let ret = rcl_init_options_set_domain_id(&mut init_options, domain_id);
        assert_eq!(ret, RCL_RET_OK as i32, "Failed to set domain_id");

        let mut context = rcl_get_zero_initialized_context();
        let ret = rcl_init(0, ptr::null(), &init_options, &mut context);
        assert_eq!(ret, RCL_RET_OK as i32, "Failed to initialize context");

        TestGetNodeNamesFixture {
            context,
            init_options,
        }
    }

    fn context(&mut self) -> *mut rcl_context_t {
        &mut self.context
    }
}

impl Drop for TestGetNodeNamesFixture {
    fn drop(&mut self) {
        let ret = rcl_shutdown(&mut self.context);
        assert_eq!(ret, RCL_RET_OK as i32, "Failed to shutdown context");

        let ret = rcl_context_fini(&mut self.context);
        assert_eq!(ret, RCL_RET_OK as i32, "Failed to finalize context");

        let ret = rcl_init_options_fini(&mut self.init_options);
        assert_eq!(ret, RCL_RET_OK as i32, "Failed to finalize init options");
    }
}

/// Helper to convert rcutils_string_array_t to Vec<String>
unsafe fn string_array_to_vec(array: &rcutils_string_array_t) -> Vec<String> {
    let mut result = Vec::new();
    for i in 0..array.size {
        // SAFETY: array.data is valid for array.size elements
        let ptr = unsafe { *array.data.add(i) };
        if !ptr.is_null() {
            // SAFETY: ptr is checked to be non-null and comes from valid C string array
            let c_str = unsafe { CStr::from_ptr(ptr) };
            result.push(c_str.to_string_lossy().to_string());
        }
    }
    result
}

/// Test rcl_get_node_names with multiple nodes
#[test]
fn test_rcl_get_node_names() {
    let mut fixture = TestGetNodeNamesFixture::new(99);

    // Expected nodes: (name, namespace)
    let mut expected_nodes: HashSet<(String, String)> = HashSet::new();

    unsafe {
        // Create node1: /node1
        let mut node1 = rcl_get_zero_initialized_node();
        let node1_name = c"node1";
        let node1_namespace = c"/";
        let node1_options = rcl_node_get_default_options();
        let ret = rcl_node_init(
            &mut node1,
            node1_name.as_ptr(),
            node1_namespace.as_ptr(),
            fixture.context(),
            &node1_options,
        );
        assert_eq!(ret, RCL_RET_OK as i32, "Failed to initialize node1");
        expected_nodes.insert(("node1".to_string(), "/".to_string()));

        // Create node2: /node2
        let mut node2 = rcl_get_zero_initialized_node();
        let node2_name = c"node2";
        let node2_namespace = c"/";
        let node2_options = rcl_node_get_default_options();
        let ret = rcl_node_init(
            &mut node2,
            node2_name.as_ptr(),
            node2_namespace.as_ptr(),
            fixture.context(),
            &node2_options,
        );
        assert_eq!(ret, RCL_RET_OK as i32, "Failed to initialize node2");
        expected_nodes.insert(("node2".to_string(), "/".to_string()));

        // Create node3: /ns/node3
        let mut node3 = rcl_get_zero_initialized_node();
        let node3_name = c"node3";
        let node3_namespace = c"/ns";
        let node3_options = rcl_node_get_default_options();
        let ret = rcl_node_init(
            &mut node3,
            node3_name.as_ptr(),
            node3_namespace.as_ptr(),
            fixture.context(),
            &node3_options,
        );
        assert_eq!(ret, RCL_RET_OK as i32, "Failed to initialize node3");
        expected_nodes.insert(("node3".to_string(), "/ns".to_string()));

        // Create node4: /ns/ns/node2 (duplicate name, different namespace)
        let mut node4 = rcl_get_zero_initialized_node();
        let node4_name = c"node2";
        let node4_namespace = c"/ns/ns";
        let node4_options = rcl_node_get_default_options();
        let ret = rcl_node_init(
            &mut node4,
            node4_name.as_ptr(),
            node4_namespace.as_ptr(),
            fixture.context(),
            &node4_options,
        );
        assert_eq!(ret, RCL_RET_OK as i32, "Failed to initialize node4");
        expected_nodes.insert(("node2".to_string(), "/ns/ns".to_string()));

        // Create node5: /node1 (duplicate name and namespace)
        let mut node5 = rcl_get_zero_initialized_node();
        let node5_name = c"node1";
        let node5_namespace = c"/";
        let node5_options = rcl_node_get_default_options();
        let ret = rcl_node_init(
            &mut node5,
            node5_name.as_ptr(),
            node5_namespace.as_ptr(),
            fixture.context(),
            &node5_options,
        );
        assert_eq!(ret, RCL_RET_OK as i32, "Failed to initialize node5");
        expected_nodes.insert(("node1".to_string(), "/".to_string()));

        // Wait for node discovery
        thread::sleep(Duration::from_secs(1));

        // Get node names
        let mut node_names = rcutils_get_zero_initialized_string_array();
        let mut node_namespaces = rcutils_get_zero_initialized_string_array();
        let ret = rcl_get_node_names(
            &node1,
            rcl_get_default_allocator(),
            &mut node_names,
            &mut node_namespaces,
        );
        assert_eq!(ret, RCL_RET_OK as i32, "Failed to get node names");

        // Verify sizes match
        assert_eq!(
            node_names.size, node_namespaces.size,
            "Node names and namespaces size mismatch"
        );

        // Convert to Vec and create HashSet of discovered nodes
        let names_vec = string_array_to_vec(&node_names);
        let namespaces_vec = string_array_to_vec(&node_namespaces);
        let mut discovered_nodes: HashSet<(String, String)> = HashSet::new();

        println!(
            "[test_rcl_get_node_names]: Found {} nodes:",
            names_vec.len()
        );
        for i in 0..names_vec.len() {
            println!("  {}/{}", namespaces_vec[i], names_vec[i]);
            discovered_nodes.insert((names_vec[i].clone(), namespaces_vec[i].clone()));
        }

        // Verify all expected nodes were discovered
        assert_eq!(
            discovered_nodes, expected_nodes,
            "Discovered nodes do not match expected nodes"
        );

        // Cleanup
        let ret = rcutils_string_array_fini(&mut node_names);
        assert_eq!(ret, RCL_RET_OK as i32);

        let ret = rcutils_string_array_fini(&mut node_namespaces);
        assert_eq!(ret, RCL_RET_OK as i32);

        let ret = rcl_node_fini(&mut node1);
        assert_eq!(ret, RCL_RET_OK as i32);

        let ret = rcl_node_fini(&mut node2);
        assert_eq!(ret, RCL_RET_OK as i32);

        let ret = rcl_node_fini(&mut node3);
        assert_eq!(ret, RCL_RET_OK as i32);

        let ret = rcl_node_fini(&mut node4);
        assert_eq!(ret, RCL_RET_OK as i32);

        let ret = rcl_node_fini(&mut node5);
        assert_eq!(ret, RCL_RET_OK as i32);
    }
}

/// Test rcl_get_node_names with invalid arguments
#[test]
fn test_rcl_get_node_names_invalid_args() {
    let mut fixture = TestGetNodeNamesFixture::new(100);

    unsafe {
        let mut node = rcl_get_zero_initialized_node();
        let node_name = c"test_node";
        let node_namespace = c"/";
        let node_options = rcl_node_get_default_options();
        let ret = rcl_node_init(
            &mut node,
            node_name.as_ptr(),
            node_namespace.as_ptr(),
            fixture.context(),
            &node_options,
        );
        assert_eq!(ret, RCL_RET_OK as i32);

        let mut node_names = rcutils_get_zero_initialized_string_array();
        let mut node_namespaces = rcutils_get_zero_initialized_string_array();

        // Test with null node
        let ret = rcl_get_node_names(
            ptr::null(),
            rcl_get_default_allocator(),
            &mut node_names,
            &mut node_namespaces,
        );
        assert_eq!(
            ret, RCL_RET_INVALID_ARGUMENT as i32,
            "Expected INVALID_ARGUMENT for null node"
        );

        // Test with null node_names
        let ret = rcl_get_node_names(
            &node,
            rcl_get_default_allocator(),
            ptr::null_mut(),
            &mut node_namespaces,
        );
        assert_eq!(
            ret, RCL_RET_INVALID_ARGUMENT as i32,
            "Expected INVALID_ARGUMENT for null node_names"
        );

        // Test with null node_namespaces
        let ret = rcl_get_node_names(
            &node,
            rcl_get_default_allocator(),
            &mut node_names,
            ptr::null_mut(),
        );
        assert_eq!(
            ret, RCL_RET_INVALID_ARGUMENT as i32,
            "Expected INVALID_ARGUMENT for null node_namespaces"
        );

        let ret = rcl_node_fini(&mut node);
        assert_eq!(ret, RCL_RET_OK as i32);
    }
}
