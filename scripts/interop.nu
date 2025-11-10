#!/usr/bin/env nu

# Interop test script for ros-z
# Tests service and pub/sub communication between ros-z and ROS2 (rmw_zenoh_cpp)

def cleanup [] {
    print "Cleaning up processes..."
    ps
        | where name =~ "z_srvcli|z_pubsub|ros2"
        | each {|x|
            print $"Killing ($x.name) \(pid: ($x.pid)\)"
            kill -9 $x.pid
        }
}

def wait_for_ready [timeout: duration = 5sec] {
    print "Waiting for nodes to be ready..."
    sleep $timeout
}

def test_service_ros_z_to_ros_z [] {
    print "\n=== Test 1: ros-z server <-> ros-z client ==="

    # Start ros-z server
    let server_job = job spawn {
        cargo run --example z_srvcli -- --mode server
    }

    wait_for_ready 2sec

    # Start ros-z client
    let result = (
        cargo run --example z_srvcli -- --mode client --a 5 --b 3
        | complete
    )

    if $result.exit_code == 0 {
        if ($result.stdout | str contains "Received response: 8") {
            print "✅ Test passed: 5 + 3 = 8"
            return true
        } else {
            print $"❌ Test failed: unexpected output ($result.stdout)"
            return false
        }
    } else {
        print $"❌ Test failed with exit code ($result.exit_code)"
        print $"Error: ($result.stderr)"
        return false
    }
}

def test_service_ros_z_server_ros2_client [] {
    print "\n=== Test 2: ros-z server <-> ROS2 client ==="

    # Start ros-z server
    let server_job = job spawn {
        cargo run --example z_srvcli -- --mode server
    }

    wait_for_ready 2sec

    # Start ROS2 client (using rmw_zenoh_cpp)
    let result = (
        with-env {RMW_IMPLEMENTATION: "rmw_zenoh_cpp"} {
            ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 10, b: 7}"
        } | complete
    )

    if $result.exit_code == 0 {
        if ($result.stdout | str contains "sum: 17") {
            print "✅ Test passed: 10 + 7 = 17"
            return true
        } else {
            print $"❌ Test failed: unexpected output ($result.stdout)"
            return false
        }
    } else {
        print $"❌ Test failed with exit code ($result.exit_code)"
        print $"Error: ($result.stderr)"
        return false
    }
}

def test_service_ros2_server_ros_z_client [] {
    print "\n=== Test 3: ROS2 server <-> ros-z client ==="

    # Start ROS2 server (using demo_nodes_cpp)
    let server_job = job spawn {
        with-env {RMW_IMPLEMENTATION: "rmw_zenoh_cpp"} {
            ros2 run demo_nodes_cpp add_two_ints_server
        }
    }

    wait_for_ready 3sec

    # Start ros-z client
    let result = (
        cargo run --example z_srvcli -- --mode client --a 15 --b 9
        | complete
    )

    if $result.exit_code == 0 {
        if ($result.stdout | str contains "Received response: 24") {
            print "✅ Test passed: 15 + 9 = 24"
            return true
        } else {
            print $"❌ Test failed: unexpected output ($result.stdout)"
            return false
        }
    } else {
        print $"❌ Test failed with exit code ($result.exit_code)"
        print $"Error: ($result.stderr)"
        return false
    }
}

def test_pubsub_ros_z_to_ros2 [] {
    print "\n=== Test 4: ros-z publisher -> ROS2 subscriber ==="

    # Start ROS2 subscriber
    let sub_job = job spawn {
        with-env {RMW_IMPLEMENTATION: "rmw_zenoh_cpp"} {
            ros2 topic echo /chatter std_msgs/msg/String --once
        }
    }

    wait_for_ready 2sec

    # Start ros-z publisher
    let result = (
        timeout 10s cargo run --example z_pubsub -- --mode pub --count 5
        | complete
    )

    # Check if subscriber received data
    let sub_result = (job list | where command =~ "ros2 topic echo")

    if ($sub_result | is-empty) {
        print "✅ Test passed: Subscriber received message and exited"
        return true
    } else {
        print "❌ Test failed: Subscriber did not receive message"
        return false
    }
}

def test_pubsub_ros2_to_ros_z [] {
    print "\n=== Test 5: ROS2 publisher -> ros-z subscriber ==="

    # Start ros-z subscriber
    let sub_output = (
        timeout 10s cargo run --example z_pubsub -- --mode sub
        | complete
    )

    # Start ROS2 publisher in background
    job spawn {
        with-env {RMW_IMPLEMENTATION: "rmw_zenoh_cpp"} {
            ros2 topic pub /chatter std_msgs/msg/String "{data: 'Hello from ROS2'}" --rate 1 --times 5
        }
    }

    wait_for_ready 3sec

    if ($sub_output.stdout | str contains "Hello from ROS2") {
        print "✅ Test passed: Received message from ROS2"
        return true
    } else {
        print "❌ Test failed: Did not receive message"
        return false
    }
}

def main [
    --test: string = "all"  # Test to run: all, service, pubsub, or specific test name
] {
    print "🧪 ros-z Interop Test Suite"
    print "============================\n"

    # Ensure we're in the right directory
    cd /home/circle/Workings/ZettaScale/project/nix-ros/ws/src/ros-z

    # Build ros-z first
    print "Building ros-z..."
    cargo build --examples
    if $env.LAST_EXIT_CODE != 0 {
        print "❌ Build failed"
        exit 1
    }
    print "✅ Build successful\n"

    # Clean up any existing processes
    cleanup

    mut results = []

    # Run tests based on selection
    if $test == "all" or $test == "service" {
        $results = ($results | append (test_service_ros_z_to_ros_z))
        cleanup
        wait_for_ready 1sec

        $results = ($results | append (test_service_ros_z_server_ros2_client))
        cleanup
        wait_for_ready 1sec

        $results = ($results | append (test_service_ros2_server_ros_z_client))
        cleanup
        wait_for_ready 1sec
    }

    if $test == "all" or $test == "pubsub" {
        $results = ($results | append (test_pubsub_ros_z_to_ros2))
        cleanup
        wait_for_ready 1sec

        $results = ($results | append (test_pubsub_ros2_to_ros_z))
        cleanup
    }

    # Final cleanup
    cleanup

    # Summary
    print "\n=============================="
    print "Test Summary"
    print "=============================="

    let passed = ($results | where $it == true | length)
    let total = ($results | length)
    let failed = $total - $passed

    print $"Total tests: ($total)"
    print $"Passed: ($passed) ✅"
    print $"Failed: ($failed) ❌"

    if $failed > 0 {
        print "\n❌ Some tests failed"
        exit 1
    } else {
        print "\n✅ All tests passed!"
        exit 0
    }
}
