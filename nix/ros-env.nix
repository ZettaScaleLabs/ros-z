# nix/ros-env.nix
{ pkgs }:
let
  distros = [
    "jazzy"
    "rolling"
  ];

  mkRosEnv =
    rosDistro: variant:
    let
      rosDeps = {
        rcl = with pkgs.rosPackages.${rosDistro}; [
          rcl
          rcl-interfaces
          rclcpp
          rcutils
          demo-nodes-py
          demo-nodes-cpp
          action-tutorials-cpp
        ];

        messages = with pkgs.rosPackages.${rosDistro}; [
          std-msgs
          geometry-msgs
          sensor-msgs
          example-interfaces
          common-interfaces
          rosidl-default-generators
          rosidl-default-runtime
          rosidl-adapter
          rosidl-typesupport-fastrtps-c
          rosidl-typesupport-fastrtps-cpp
        ];

        testMessages = with pkgs.rosPackages.${rosDistro}; [
          test-msgs
        ];

        devExtras = with pkgs.rosPackages.${rosDistro}; [
          ament-cmake-core
          ros-core
          rclpy
          rmw
          rmw-implementation
          rmw-zenoh-cpp
          ament-cmake
          ament-cmake-gtest
          ament-lint-auto
          ament-lint-common
          launch
          launch-testing
          ros2cli
        ];
      };
    in
    pkgs.rosPackages.${rosDistro}.buildEnv {
      paths =
        {
          dev = rosDeps.rcl ++ rosDeps.messages ++ rosDeps.testMessages ++ rosDeps.devExtras;
          rcl = rosDeps.rcl;
          msgs = rosDeps.messages;
          build = rosDeps.rcl ++ rosDeps.messages;
          testCore = rosDeps.rcl ++ rosDeps.messages;
          testFull = rosDeps.rcl ++ rosDeps.messages ++ rosDeps.testMessages;
        }
        .${variant};
    };
in
{
  inherit distros mkRosEnv;
}
