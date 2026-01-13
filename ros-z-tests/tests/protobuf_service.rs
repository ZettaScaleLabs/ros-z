mod common;

use std::{thread, time::Duration};

use common::*;
use ros_z::{
    Builder, MessageTypeInfo, ServiceTypeInfo, WithTypeInfo, entity::TypeHash,
    msg::{ProtobufSerdes, ZMessage, ZService},
};

// Define protobuf test messages using prost
use prost::Message as ProstMessage;

#[derive(Clone, PartialEq, ProstMessage)]
pub struct TestRequest {
    #[prost(int32, tag = "1")]
    pub a: i32,
    #[prost(int32, tag = "2")]
    pub b: i32,
}

#[derive(Clone, PartialEq, ProstMessage)]
pub struct TestResponse {
    #[prost(int32, tag = "1")]
    pub sum: i32,
}

// Implement required traits for TestRequest
impl MessageTypeInfo for TestRequest {
    fn type_name() -> &'static str {
        "test::srv::dds_::TestAdd_Request_"
    }

    fn type_hash() -> TypeHash {
        TypeHash::zero()
    }
}

impl WithTypeInfo for TestRequest {}

impl ZMessage for TestRequest {
    type Serdes = ProtobufSerdes<TestRequest>;
}

// Implement required traits for TestResponse
impl MessageTypeInfo for TestResponse {
    fn type_name() -> &'static str {
        "test::srv::dds_::TestAdd_Response_"
    }

    fn type_hash() -> TypeHash {
        TypeHash::zero()
    }
}

impl WithTypeInfo for TestResponse {}

impl ZMessage for TestResponse {
    type Serdes = ProtobufSerdes<TestResponse>;
}

// Service type definition
pub struct TestAdd;

impl ServiceTypeInfo for TestAdd {
    fn service_type_info() -> ros_z::entity::TypeInfo {
        ros_z::entity::TypeInfo::new("test::srv::dds_::TestAdd_", TypeHash::zero())
    }
}

impl ZService for TestAdd {
    type Request = TestRequest;
    type Response = TestResponse;
}

#[test]
fn test_protobuf_service_ros_z_to_ros_z() {
    let router = TestRouter::new();

    println!("\n=== Test: Protobuf Service (ros-z server <-> ros-z client) ===");

    // Start server in background thread
    let router_endpoint = router.endpoint().to_string();
    let _server_handle = thread::spawn(move || {
        let ctx =
            create_ros_z_context_with_endpoint(&router_endpoint).expect("Failed to create context");

        let node = ctx
            .create_node("protobuf_test_server")
            .build()
            .expect("Failed to create node");

        let mut server = node
            .create_service::<TestAdd>("/test_protobuf_add")
            .build()
            .expect("Failed to create service");

        println!("[Server] Ready, waiting for requests...");

        // Handle one request
        if let Ok((key, req)) = server.take_request() {
            println!("[Server] Received request: {} + {}", req.a, req.b);
            let resp = TestResponse { sum: req.a + req.b };
            println!("[Server] Sending response: {}", resp.sum);
            server
                .send_response(&resp, &key)
                .expect("Failed to send response");
        }
    });

    // Give server time to start
    thread::sleep(Duration::from_millis(500));

    // Create client
    let ctx = create_ros_z_context_with_endpoint(router.endpoint()).expect("Failed to create context");

    let node = ctx
        .create_node("protobuf_test_client")
        .build()
        .expect("Failed to create node");

    let client = node
        .create_client::<TestAdd>("/test_protobuf_add")
        .build()
        .expect("Failed to create client");

    println!("[Client] Sending request: 10 + 32");

    let request = TestRequest { a: 10, b: 32 };

    // Send request and wait for response
    let response = tokio::runtime::Runtime::new()
        .unwrap()
        .block_on(async {
            client.send_request(&request).await?;
            client.take_response_timeout(Duration::from_secs(2))
        })
        .expect("Failed to get response");

    println!("[Client] Received response: {}", response.sum);

    assert_eq!(response.sum, 42, "Expected 10 + 32 = 42");

    println!("✓ Protobuf service test passed!");
}

#[test]
fn test_protobuf_service_multiple_calls() {
    let router = TestRouter::new();

    println!("\n=== Test: Protobuf Service Multiple Calls ===");

    // Start server in background thread
    let router_endpoint = router.endpoint().to_string();
    let _server_handle = thread::spawn(move || {
        let ctx =
            create_ros_z_context_with_endpoint(&router_endpoint).expect("Failed to create context");

        let node = ctx
            .create_node("protobuf_multi_server")
            .build()
            .expect("Failed to create node");

        let mut server = node
            .create_service::<TestAdd>("/test_protobuf_multi")
            .build()
            .expect("Failed to create service");

        println!("[Server] Ready for multiple requests...");

        // Handle 3 requests
        for i in 0..3 {
            if let Ok((key, req)) = server.take_request() {
                println!("[Server] Request {}: {} + {}", i + 1, req.a, req.b);
                let resp = TestResponse { sum: req.a + req.b };
                server
                    .send_response(&resp, &key)
                    .expect("Failed to send response");
            }
        }
    });

    // Give server time to start
    thread::sleep(Duration::from_millis(500));

    // Create client
    let ctx = create_ros_z_context_with_endpoint(router.endpoint()).expect("Failed to create context");

    let node = ctx
        .create_node("protobuf_multi_client")
        .build()
        .expect("Failed to create node");

    let client = node
        .create_client::<TestAdd>("/test_protobuf_multi")
        .build()
        .expect("Failed to create client");

    let test_cases = vec![(5, 7, 12), (100, 200, 300), (15, 27, 42)];

    for (a, b, expected) in test_cases {
        println!("[Client] Sending: {} + {}", a, b);

        let request = TestRequest { a, b };

        let response = tokio::runtime::Runtime::new()
            .unwrap()
            .block_on(async {
                client.send_request(&request).await?;
                client.take_response_timeout(Duration::from_secs(2))
            })
            .expect("Failed to get response");

        println!("[Client] Response: {}", response.sum);
        assert_eq!(response.sum, expected, "Expected {} + {} = {}", a, b, expected);

        thread::sleep(Duration::from_millis(100));
    }

    println!("✓ Multiple calls test passed!");
}
