#!/usr/bin/env python3
"""Test complex message types with nested structures."""

import ros_z_py
import time

def test_vector3(node):
    """Test geometry_msgs/Vector3 serialization."""
    print("Testing Vector3...")

    # Create publisher and subscriber
    pub = node.create_publisher("/vector", "geometry_msgs/msg/Vector3")
    sub = node.create_subscriber("/vector", "geometry_msgs/msg/Vector3")

    time.sleep(0.5)

    # Test data
    test_msg = {"x": 1.0, "y": 2.0, "z": 3.0}
    pub.publish(test_msg)
    print(f"  Published: {test_msg}")

    # Receive and validate
    msg = sub.recv(timeout=2.0)
    assert msg is not None, "No message received"
    assert msg["x"] == 1.0
    assert msg["y"] == 2.0
    assert msg["z"] == 3.0
    print(f"  Received: {msg}")
    print("✓ Vector3 works")

def test_twist(node):
    """Test geometry_msgs/Twist serialization (nested message)."""
    print("Testing Twist (nested message)...")

    # Create publisher and subscriber
    pub = node.create_publisher("/cmd_vel", "geometry_msgs/msg/Twist")
    sub = node.create_subscriber("/cmd_vel", "geometry_msgs/msg/Twist")

    time.sleep(0.5)

    # Test data with nested Vector3 messages
    test_msg = {
        "linear": {"x": 1.0, "y": 0.0, "z": 0.0},
        "angular": {"x": 0.0, "y": 0.0, "z": 0.5}
    }
    pub.publish(test_msg)
    print(f"  Published: {test_msg}")

    # Receive and validate
    msg = sub.recv(timeout=2.0)
    assert msg is not None, "No message received"
    assert msg["linear"]["x"] == 1.0
    assert msg["linear"]["y"] == 0.0
    assert msg["linear"]["z"] == 0.0
    assert msg["angular"]["x"] == 0.0
    assert msg["angular"]["y"] == 0.0
    assert msg["angular"]["z"] == 0.5
    print(f"  Received: {msg}")
    print("✓ Twist works")

def main():
    print("=" * 60)
    print("ros-z-py Complex Messages Test")
    print("=" * 60)

    # Setup
    session = ros_z_py.open_session(domain_id=0)
    node = ros_z_py.create_node(session, "test_complex_node", "/test")

    # Run tests
    test_vector3(node)
    test_twist(node)

    print("=" * 60)
    print("All tests passed! ✓")
    print("=" * 60)

if __name__ == "__main__":
    main()