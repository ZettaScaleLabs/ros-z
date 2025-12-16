{
  description = "ros-z: Native Rust ROS 2 implementation using Zenoh";

  inputs = {
    nix-ros-overlay.url = "github:lopsided98/nix-ros-overlay";
    nixpkgs.follows = "nix-ros-overlay/nixpkgs";
    rust-overlay.url = "github:oxalica/rust-overlay";
    git-hooks.url = "github:cachix/git-hooks.nix";
    systems.url = "github:nix-systems/default";
  };

  outputs =
    {
      self,
      nix-ros-overlay,
      nixpkgs,
      rust-overlay,
      git-hooks,
      systems,
    }:
    nix-ros-overlay.inputs.flake-utils.lib.eachDefaultSystem (
      system:
      let
        # List of supported ROS distros
        distros = [
          "jazzy" # default
          "rolling"
        ];

        pkgs = import nixpkgs {
          inherit system;
          overlays = [
            nix-ros-overlay.overlays.default
            rust-overlay.overlays.default
          ];
        };

        rustToolchain = pkgs.rust-bin.stable."1.91.0".default.override {
          extensions = [
            "rust-src"
            "rust-analyzer"
          ];
        };

        rustfmtNightly = pkgs.rust-bin.nightly.latest.rustfmt;

        # Override rustfmt to use nightly
        rustfmt-nightly-bin = pkgs.writeShellScriptBin "rustfmt" ''
          exec ${rustfmtNightly}/bin/rustfmt "$@"
        '';

        # Factory to create environment for a specific ROS distro
        mkRosEnv =
          rosDistro:
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

              # Test-only message packages
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
          {
            # Development environment with all dependencies including test messages
            dev = pkgs.rosPackages.${rosDistro}.buildEnv {
              paths = rosDeps.rcl ++ rosDeps.messages ++ rosDeps.testMessages ++ rosDeps.devExtras;
            };

            # Core RCL only (for minimal builds)
            rcl = pkgs.rosPackages.${rosDistro}.buildEnv {
              paths = rosDeps.rcl;
            };

            # Runtime messages only
            msgs = pkgs.rosPackages.${rosDistro}.buildEnv {
              paths = rosDeps.messages;
            };

            # Build environment with runtime messages but NO test messages
            build = pkgs.rosPackages.${rosDistro}.buildEnv {
              paths = rosDeps.rcl ++ rosDeps.messages;
            };

            # Test environment for core tests only (no test_msgs)
            testCore = pkgs.rosPackages.${rosDistro}.buildEnv {
              paths = rosDeps.rcl ++ rosDeps.messages;
            };

            # Test environment with test messages (for all tests)
            testFull = pkgs.rosPackages.${rosDistro}.buildEnv {
              paths = rosDeps.rcl ++ rosDeps.messages ++ rosDeps.testMessages;
            };
          };

        # Common build tools
        commonBuildInputs = with pkgs; [
          rustToolchain
          sccache
          clang
          llvmPackages.libclang
          llvmPackages.bintools
          pkg-config
          nushell
          protobuf
          markdownlint-cli
        ];

        # Development tools
        devTools = with pkgs; [
          cargo-edit
          cargo-watch
          clang-tools
          rust-analyzer
          nixfmt-rfc-style
          gdb
          mdbook
        ];

        # Test tools
        testTools = with pkgs; [
          cargo-nextest
        ];

        # Environment variables for Rust/C++ interop
        commonEnvVars = rec {
          LIBCLANG_PATH = "${pkgs.llvmPackages.libclang.lib}/lib";
          CLANG_PATH = "${pkgs.llvmPackages.clang}/bin/clang";
          LD_LIBRARY_PATH = pkgs.lib.makeLibraryPath [
            pkgs.stdenv.cc.cc.lib
            # ROS libraries will be added by the ROS environment packages
          ];
          RUST_BACKTRACE = "1";
          RMW_IMPLEMENTATION = "rmw_zenoh_cpp";
          RUSTC_WRAPPER = "${pkgs.sccache}/bin/sccache";
        };

        # Export environment variables as shell commands
        exportEnvVars = pkgs.lib.concatStringsSep "\n" (
          pkgs.lib.mapAttrsToList (name: value: "export ${name}=\"${value}\"") commonEnvVars
        );

        # Base shell configuration factory
        mkDevShell =
          {
            name,
            packages,
            banner ? "",
            extraShellHook ? "",
          }:
          pkgs.mkShell {
            inherit name packages;
            shellHook = ''
              ${exportEnvVars}
              ${extraShellHook}
              ${if banner != "" then banner else ""}
            '';
            hardeningDisable = [ "all" ];
          };

        # Helper to create shells for a specific ROS distro
        mkRosShells =
          rosDistro:
          let
            rosEnv = mkRosEnv rosDistro;
          in
          {
            default = mkDevShell {
              name = "ros-z-dev-${rosDistro}";
              packages = [
                rustfmt-nightly-bin
              ]
              ++ commonBuildInputs
              ++ devTools
              ++ testTools
              ++ [ rosEnv.dev ]
              ++ pre-commit-check.enabledPackages;
              extraShellHook = pre-commit-check.shellHook;
              banner = ''
                echo "🦀 ros-z development environment (with ROS)"
                echo "ROS 2 Distribution: ${rosDistro}"
                echo "Rust: $(rustc --version)"
              '';
            };

            ci = mkDevShell {
              name = "ros-z-ci-${rosDistro}";
              packages = commonBuildInputs ++ testTools ++ [ rosEnv.testFull ];
            };
          };

        # Generate shells for all distros
        allDistroShells = builtins.listToAttrs (
          builtins.map (distro: {
            name = distro;
            value = mkRosShells distro;
          }) distros
        );
        # Pre-commit hooks configuration
        pre-commit-check = import ./nix/pre-commit.nix {
          inherit
            pkgs
            git-hooks
            system
            rustfmtNightly
            ;
        };
      in
      {
        # Pre-commit checks
        checks = {
          inherit pre-commit-check;
        };

        # Development shells
        devShells = {
          # Default: first distro in the list with ROS
          default = allDistroShells.${builtins.head distros}.default;

          # Without ROS
          pureRust = mkDevShell {
            name = "ros-z-pure-rust";
            packages = [
              rustfmt-nightly-bin
            ]
            ++ commonBuildInputs
            ++ devTools
            ++ testTools
            ++ pre-commit-check.enabledPackages;
            extraShellHook = pre-commit-check.shellHook;
            banner = ''
              echo "🦀 ros-z development environment (pure Rust)"
              echo "Rust: $(rustc --version)"
            '';
          };

          # CI without ROS
          pureRust-ci = mkDevShell {
            name = "ros-z-ci-pure-rust";
            packages = commonBuildInputs ++ testTools;
          };
        }
        # Add per-distro dev shells (ros-jazzy, ros-rolling, ...)
        // (builtins.listToAttrs (
          builtins.map (distro: {
            name = "ros-${distro}";
            value = allDistroShells.${distro}.default;
          }) distros
        ))
        # Add per-distro CI shells (ros-jazzy-ci, ros-rolling-ci, ...)
        // (builtins.listToAttrs (
          builtins.map (distro: {
            name = "ros-${distro}-ci";
            value = allDistroShells.${distro}.ci;
          }) distros
        ));

        formatter = pkgs.nixfmt-rfc-style;
      }
    );

  nixConfig = {
    extra-substituters = [ "https://ros.cachix.org" ];
    extra-trusted-public-keys = [ "ros.cachix.org-1:dSyZxI8geDCJrwgvCOHDoAfOm5sV1wCPjBkKL+38Rvo=" ];
  };
}
