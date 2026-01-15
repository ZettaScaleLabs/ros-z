#!/usr/bin/env python3
"""Twist publisher example - publishes velocity commands."""

import ros_z_py
import time
import math

def main():
    # Create session and node
    session = ros_z_py.open_session(domain_id=0)
    node = ros_z_py.create_node(session, "twist_publisher", "/")

    # Create publisher
    pub = node.create_publisher("/cmd_vel", "geometry_msgs/msg/Twist")

    print("Publishing Twist messages to /cmd_vel...")

    t = 0.0
    while True:
        # Create circular motion command
        twist = {
            "linear": {
                "x": 0.5,
                "y": 0.0,
                "z": 0.0
            },
            "angular": {
                "x": 0.0,
                "y": 0.0,
                "z": 0.5 * math.sin(t)
            }
        }

        pub.publish(twist)
        print(f"Published: linear.x={twist['linear']['x']:.2f}, angular.z={twist['angular']['z']:.2f}")

        t += 0.1
        time.sleep(0.1)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nTwist publisher stopped.")