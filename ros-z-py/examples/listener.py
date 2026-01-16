#!/usr/bin/env python3
"""Simple listener example - subscribes to String messages."""

import ros_z_py


def main():
    # Create session and node
    session = ros_z_py.open_session(domain_id=0)
    node = ros_z_py.create_node(session, "listener", "/")

    # Create subscriber
    sub = node.create_subscriber("/chatter", "std_msgs/msg/String")

    print("Listener started. Waiting for messages on /chatter...")

    while True:
        msg = sub.try_recv()
        if msg:
            print(f"Received: {msg.data}")
        else:
            import time

            time.sleep(0.1)  # Small delay to prevent busy waiting


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nListener stopped.")
