#!/usr/bin/env python3
"""
Simple publisher/subscriber example for ros-z Python bindings.

This example demonstrates:
1. Creating a ROS 2 session and node
2. Publishing std_msgs/String messages
3. Subscribing to std_msgs/String messages
4. Using QoS profiles

Note: This requires the ros-z-py module to be built with maturin:
    cd ros-z-py
    maturin develop --release
"""

import ros_z_py
import time

def main():
    # Create a ROS 2 session (domain_id=0 is default)
    session = ros_z_py.open_session(domain_id=0)
    print("✓ Session opened")

    # Create a node
    node = ros_z_py.create_node(session, "python_example", namespace="/")
    print("✓ Node created: python_example")

    # Create a publisher for std_msgs/String
    # Note: create_publisher is currently a placeholder - needs full implementation
    # This will be functional once Phase 1.8 publisher/subscriber creation is complete
    try:
        pub = node.create_publisher(
            "/chatter",
            "std_msgs/msg/String",
            qos={
                "reliability": "reliable",
                "history": "keep_last",
                "depth": 10
            }
        )
        print("✓ Publisher created on /chatter")

        # Publish a message
        msg = {"data": "Hello from Python!"}
        pub.publish(msg)
        print(f"✓ Published: {msg}")

        # Create a subscriber
        sub = node.create_subscriber("/chatter", "std_msgs/msg/String")
        print("✓ Subscriber created on /chatter")

        # Receive messages
        print("Waiting for messages (timeout=1.0s)...")
        received_msg = sub.recv(timeout=1.0)
        if received_msg:
            print(f"✓ Received: {received_msg}")
        else:
            print("✗ No message received (timeout)")

    except NotImplementedError as e:
        print(f"\n⚠ {e}")
        print("\nNote: Publisher/Subscriber creation requires implementing")
        print("      the actual ZPub/ZSub instantiation in PyNode methods.")
        print("      The registry and serialization are ready!")


def example_twist():
    """Example for geometry_msgs/Twist messages."""
    session = ros_z_py.open_session(domain_id=0)
    node = ros_z_py.create_node(session, "twist_example")

    try:
        pub = node.create_publisher("/cmd_vel", "geometry_msgs/msg/Twist")

        # Publish a Twist message
        msg = {
            "linear": {"x": 0.5, "y": 0.0, "z": 0.0},
            "angular": {"x": 0.0, "y": 0.0, "z": 0.3}
        }
        pub.publish(msg)
        print(f"✓ Published Twist: linear.x={msg['linear']['x']}, angular.z={msg['angular']['z']}")

    except NotImplementedError as e:
        print(f"⚠ {e}")


if __name__ == "__main__":
    print("ros-z Python Bindings Example")
    print("=" * 50)
    main()
    print("\n" + "=" * 50)
    print("\nTwist Example:")
    example_twist()
