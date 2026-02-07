#!/usr/bin/env python3
"""Test basic pub/sub functionality with std_msgs/String."""

import ros_z_py
from ros_z_py import std_msgs
import time


def test_context_creation():
    """Test creating a context."""
    print("Testing context creation...")
    context = ros_z_py.ZContextBuilder().with_domain_id(0).build()
    assert context is not None
    print("✓ Context created")
    return context


def test_node_creation(context):
    """Test creating a node."""
    print("Testing node creation...")
    node = context.create_node("test_node").with_namespace("/test").build()
    assert node is not None
    print("✓ Node created")
    return node


def test_publisher_creation(node):
    """Test creating a publisher."""
    print("Testing publisher creation...")
    pub = node.create_publisher("/chatter", std_msgs.String)
    assert pub is not None
    print("✓ Publisher created")
    return pub


def test_subscriber_creation(node):
    """Test creating a subscriber."""
    print("Testing subscriber creation...")
    sub = node.create_subscriber("/chatter", std_msgs.String)
    assert sub is not None
    print("✓ Subscriber created")
    return sub


def test_publish_receive(pub, sub):
    """Test publishing and receiving a message."""
    print("Testing publish/receive...")

    # Give subscriber time to set up
    time.sleep(0.5)

    # Publish message
    test_data = "Hello, ros-z-py!"
    msg_to_send = std_msgs.String(data=test_data)
    pub.publish(msg_to_send)
    print(f"  Published: {test_data}")

    # Receive message
    msg = sub.recv(timeout=2.0)
    assert msg is not None, "No message received"
    assert hasattr(msg, "data"), "Message missing 'data' field"
    assert msg.data == test_data, f"Expected '{test_data}', got '{msg.data}'"
    print(f"  Received: {msg.data}")
    print("✓ Publish/receive works")


def test_qos_configuration(node):
    """Test QoS configuration."""
    print("Testing QoS configuration...")

    qos = {
        "reliability": "reliable",
        "durability": "volatile",
        "history": "keep_last",
        "depth": 10,
    }

    pub = node.create_publisher("/qos_topic", std_msgs.String, qos=qos)
    assert pub is not None
    print("✓ QoS configuration accepted")


def test_error_handling(node):
    """Test error handling for invalid message types."""
    print("Testing error handling...")

    # Passing a non-class type should raise TypeError
    try:
        _ = node.create_publisher("/test", "not_a_class")  # type: ignore
        assert False, "Should have raised error for invalid type"
    except TypeError:
        print("✓ Error handling works (invalid type rejected)")


def main():
    print("=" * 60)
    print("ros-z-py Basic Pub/Sub Test")
    print("=" * 60)

    # Run tests
    context = test_context_creation()
    node = test_node_creation(context)
    pub = test_publisher_creation(node)
    sub = test_subscriber_creation(node)
    test_publish_receive(pub, sub)
    test_qos_configuration(node)
    test_error_handling(node)

    print("=" * 60)
    print("All tests passed! ✓")
    print("=" * 60)


if __name__ == "__main__":
    main()
