#!/usr/bin/env python3
"""Simple talker example - publishes String messages."""

import ros_z_py
from ros_z_py import std_msgs
import time

def main():
    # Create session and node
    session = ros_z_py.open_session(domain_id=0)
    node = ros_z_py.create_node(session, "talker", "/")

    # Create publisher
    pub = node.create_publisher("/chatter", "std_msgs/msg/String")

    print("Talker started. Publishing to /chatter...")

    count = 0
    while True:
        message = f"Hello, World! {count}"
        msg = std_msgs.String(data=message)
        pub.publish(msg)
        print(f"Published: {message}")
        count += 1
        time.sleep(1.0)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nTalker stopped.")