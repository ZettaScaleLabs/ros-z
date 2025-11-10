{
  description = "ros-z: Native Rust ROS2 implementation using Zenoh";

  inputs = {
    nix-ros-overlay.url = "github:lopsided98/nix-ros-overlay";
    nixpkgs.follows = "nix-ros-overlay/nixpkgs";
    rust-overlay.url = "github:oxalica/rust-overlay";
  };

  outputs =
    {
      self,
      nix-ros-overlay,
      nixpkgs,
      rust-overlay,
    }:
    nix-ros-overlay.inputs.flake-utils.lib.eachDefaultSystem (
      system:
      let
        pkgs = import nixpkgs {
          inherit system;
          overlays = [
            nix-ros-overlay.overlays.default
            (import rust-overlay)
          ];
        };

        rust = pkgs.rust-bin.stable.latest.default.override {
          extensions = [ "rust-src" "rust-analyzer" ];
        };

        rosDistro = "rolling";

        # Core ROS packages needed for ros-z development and testing
        coreRosPackages = [
          "ament-cmake-core"
          "ros-core"
          "demo-nodes-cpp"
          "demo-nodes-py"
        ];

        # RCL packages
        rclPackages = [
          "rcl"
          "rcl-interfaces"
          "rclcpp"
          "rclpy"
          "rcutils"
        ];

        # RMW packages - only rmw_zenoh_cpp
        rmwPackages = [
          "rmw"
          "rmw-implementation"
          "rmw-zenoh-cpp"
        ];

        # ROSIDL packages for message generation
        rosidlPackages = [
          "rosidl-default-generators"
          "rosidl-default-runtime"
          "rosidl-adapter"
        ];

        # Message packages
        messagePackages = [
          "std-msgs"
          "geometry-msgs"
          "sensor-msgs"
          "example-interfaces"
          "common-interfaces"
        ];

        # Testing packages
        testingPackages = [
          "ament-cmake"
          "ament-cmake-gtest"
          "ament-lint-auto"
          "ament-lint-common"
          "launch"
          "launch-testing"
        ];

        # CLI tools
        cliPackages = [
          "ros2cli"
        ];

        # Function to create ROS package list
        makeRosPackages =
          distro:
          let
            allPackages =
              coreRosPackages
              ++ rclPackages
              ++ rmwPackages
              ++ rosidlPackages
              ++ messagePackages
              ++ testingPackages
              ++ cliPackages;
          in
          with pkgs.rosPackages.${distro};
          buildEnv {
            paths = map (pkg: pkgs.rosPackages.${distro}.${pkg}) allPackages;
          };

        # Common build inputs
        commonBuildInputs = with pkgs; [
          rust
          cargo-edit
          cargo-watch
          clang
          llvmPackages.libclang
          pkg-config
          nushell
        ];

        # Development dependencies
        devDependencies = with pkgs; [
          clang-tools
          rust-analyzer
        ];

        LD_LIBRARY_PATH =
          with pkgs;
          lib.makeLibraryPath [
            stdenv.cc.cc.lib
          ];
      in
      {
        # Default development shell
        devShells.default = pkgs.mkShell {
          name = "ros-z-dev";
          packages = commonBuildInputs ++ devDependencies ++ [
            (makeRosPackages rosDistro)
          ];

          hardeningDisable = [ "all" ];

          shellHook = ''
            export LIBCLANG_PATH="${pkgs.llvmPackages.libclang.lib}/lib"
            export CLANG_PATH="${pkgs.llvmPackages.clang}/bin/clang"
            export LD_LIBRARY_PATH="${LD_LIBRARY_PATH}"
            export RUST_BACKTRACE=1

            # Use rmw_zenoh_cpp by default
            export RMW_IMPLEMENTATION=rmw_zenoh_cpp

            # ROS2 setup
            # Note: nix-ros-overlay automatically sets up ROS environment

            echo "🦀 ros-z development environment"
            echo "ROS2 Distribution: ${rosDistro}"
            echo "RMW Implementation: rmw_zenoh_cpp"
            echo "Rust: $(rustc --version)"
            echo ""
            echo "Available commands:"
            echo "  cargo build                    - Build ros-z"
            echo "  cargo test                     - Run tests"
            echo "  cargo check --all-features     - Check all features"
            echo "  nu scripts/interop.nu          - Run interop tests"
          '';

          inherit LD_LIBRARY_PATH;
        };

        # CI shell for GitHub Actions
        devShells.ci = pkgs.mkShell {
          name = "ros-z-ci";
          packages = commonBuildInputs ++ [
            (makeRosPackages rosDistro)
          ];

          shellHook = ''
            export LIBCLANG_PATH="${pkgs.llvmPackages.libclang.lib}/lib"
            export CLANG_PATH="${pkgs.llvmPackages.clang}/bin/clang"
            export LD_LIBRARY_PATH="${LD_LIBRARY_PATH}"
            export RUST_BACKTRACE=1
            export RMW_IMPLEMENTATION=rmw_zenoh_cpp

            echo "ros-z CI environment ready"
          '';

          inherit LD_LIBRARY_PATH;
        };
      }
    );

  nixConfig = {
    extra-substituters = "https://ros.cachix.org";
    extra-trusted-public-keys = "ros.cachix.org-1:dSyZxI8geDCJrwgvCOHDoAfOm5sV1wCPjBkKL+38Rvo=";
  };
}
