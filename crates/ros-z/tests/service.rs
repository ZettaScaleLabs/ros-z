use std::{thread, time::Duration};

use ros_z::{
    Builder, MessageTypeInfo, ServiceTypeInfo, context::ZContextBuilder, entity::TypeHash,
    msg::ZService,
};
use serde::{Deserialize, Serialize};
use serde_json::json;

// Simple test service request
#[derive(Debug, Clone, Serialize, Deserialize, Default, PartialEq)]
struct AddTwoIntsRequest {
    a: i64,
    b: i64,
}

impl MessageTypeInfo for AddTwoIntsRequest {
    fn type_name() -> &'static str {
        "test_msgs::srv::dds_::AddTwoInts_Request_"
    }

    fn type_hash() -> TypeHash {
        TypeHash::zero()
    }
}

impl ros_z::WithTypeInfo for AddTwoIntsRequest {}

// Simple test service response
#[derive(Debug, Clone, Serialize, Deserialize, Default, PartialEq)]
struct AddTwoIntsResponse {
    sum: i64,
}

impl MessageTypeInfo for AddTwoIntsResponse {
    fn type_name() -> &'static str {
        "test_msgs::srv::dds_::AddTwoInts_Response_"
    }

    fn type_hash() -> TypeHash {
        TypeHash::zero()
    }
}

impl ros_z::WithTypeInfo for AddTwoIntsResponse {}

// Service type definition
struct AddTwoInts;

impl ServiceTypeInfo for AddTwoInts {
    fn service_type_info() -> ros_z::entity::TypeInfo {
        ros_z::entity::TypeInfo::new("test_msgs::srv::dds_::AddTwoInts_", TypeHash::zero())
    }
}

impl ZService for AddTwoInts {
    type Request = AddTwoIntsRequest;
    type Response = AddTwoIntsResponse;
}

#[test]
fn test_basic_service_request_response() {
    let ctx = ZContextBuilder::default()
        .disable_multicast_scouting()
        .with_json("connect/endpoints", json!([]))
        .build()
        .expect("Failed to create context");

    let server_handle = thread::spawn({
        let ctx = ctx.clone();
        move || {
            let node = ctx
                .create_node("test_server")
                .build()
                .expect("Failed to create node");

            let mut server = node
                .create_service::<AddTwoInts>("add_two_ints")
                .build()
                .expect("Failed to create server");

            // Wait for request
            let (key, request) = server.take_request().expect("Failed to take request");
            assert_eq!(request.a, 10);
            assert_eq!(request.b, 32);

            let response = AddTwoIntsResponse {
                sum: request.a + request.b,
            };
            server
                .send_response(&response, &key)
                .expect("Failed to send response");
        }
    });

    let client_handle = thread::spawn({
        let ctx = ctx.clone();
        move || {
            let node = ctx
                .create_node("test_client")
                .build()
                .expect("Failed to create node");

            let client = node
                .create_client::<AddTwoInts>("add_two_ints")
                .build()
                .expect("Failed to create client");

            // Give server time to start
            thread::sleep(Duration::from_millis(100));

            let request = AddTwoIntsRequest { a: 10, b: 32 };

            tokio::runtime::Runtime::new()
                .unwrap()
                .block_on(async { client.send_request(&request).await })
                .expect("Failed to send request");

            let response = client
                .take_response_timeout(Duration::from_secs(2))
                .expect("Failed to receive response");

            assert_eq!(response.sum, 42);
        }
    });

    server_handle.join().expect("Server thread panicked");
    client_handle.join().expect("Client thread panicked");
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn test_async_service_request_response() {
    let ctx = ZContextBuilder::default()
        .disable_multicast_scouting()
        .with_json("connect/endpoints", json!([]))
        .build()
        .expect("Failed to create context");

    let server_ctx = ctx.clone();
    let server_handle = tokio::spawn(async move {
        let node = server_ctx
            .create_node("async_server")
            .build()
            .expect("Failed to create node");

        let mut server = node
            .create_service::<AddTwoInts>("async_add")
            .build()
            .expect("Failed to create server");

        // Wait for request asynchronously
        let (key, request) = server
            .take_request_async()
            .await
            .expect("Failed to take request");

        let response = AddTwoIntsResponse {
            sum: request.a + request.b,
        };

        server
            .send_response_async(&response, &key)
            .await
            .expect("Failed to send response");
    });

    let client_ctx = ctx.clone();
    let client_handle = tokio::spawn(async move {
        let node = client_ctx
            .create_node("async_client")
            .build()
            .expect("Failed to create node");

        let client = node
            .create_client::<AddTwoInts>("async_add")
            .build()
            .expect("Failed to create client");

        // Give server time to start
        tokio::time::sleep(Duration::from_millis(100)).await;

        let request = AddTwoIntsRequest { a: 100, b: 23 };
        client
            .send_request(&request)
            .await
            .expect("Failed to send request");

        let response = client
            .take_response_async()
            .await
            .expect("Failed to receive response");

        assert_eq!(response.sum, 123);
    });

    let (server_result, client_result) = tokio::join!(server_handle, client_handle);
    server_result.expect("Server task panicked");
    client_result.expect("Client task panicked");
}

#[test]
fn test_multiple_service_requests() {
    let ctx = ZContextBuilder::default()
        .disable_multicast_scouting()
        .with_json("connect/endpoints", json!([]))
        .build()
        .expect("Failed to create context");

    let server_handle = thread::spawn({
        let ctx = ctx.clone();
        move || {
            let node = ctx
                .create_node("multi_server")
                .build()
                .expect("Failed to create node");

            let mut server = node
                .create_service::<AddTwoInts>("multi_add")
                .build()
                .expect("Failed to create server");

            // Handle 3 requests
            for expected_a in [1, 2, 3] {
                let (key, request) = server.take_request().expect("Failed to take request");
                assert_eq!(request.a, expected_a);
                assert_eq!(request.b, 10);

                let response = AddTwoIntsResponse {
                    sum: request.a + request.b,
                };
                server
                    .send_response(&response, &key)
                    .expect("Failed to send response");
            }
        }
    });

    let client_handle = thread::spawn({
        let ctx = ctx.clone();
        move || {
            let node = ctx
                .create_node("multi_client")
                .build()
                .expect("Failed to create node");

            let client = node
                .create_client::<AddTwoInts>("multi_add")
                .build()
                .expect("Failed to create client");

            // Give server time to start
            thread::sleep(Duration::from_millis(100));

            let rt = tokio::runtime::Runtime::new().unwrap();

            // Send 3 requests sequentially
            for a in [1, 2, 3] {
                let request = AddTwoIntsRequest { a, b: 10 };

                rt.block_on(async { client.send_request(&request).await })
                    .expect("Failed to send request");

                let response = client
                    .take_response_timeout(Duration::from_secs(2))
                    .expect("Failed to receive response");

                assert_eq!(response.sum, a + 10);
            }
        }
    });

    server_handle.join().expect("Server thread panicked");
    client_handle.join().expect("Client thread panicked");
}
