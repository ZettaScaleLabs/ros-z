{
  description = "ros-z: Native Rust ROS 2 implementation using Zenoh";

  inputs = {
    nix-ros-overlay.url = "github:lopsided98/nix-ros-overlay";
    nixpkgs.follows = "nix-ros-overlay/nixpkgs"; # IMPORTANT!!!
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
            rust-overlay.overlays.default
          ];
        };

        rustToolchain = pkgs.rust-bin.stable."1.91.0".default.override {
          extensions = [
            "rust-src"
            "rust-analyzer"
          ];
        };

        rosDistro = "rolling";

        # Define ROS package dependencies per workspace member
        rosDeps = {
          # rcl-z needs RCL packages
          rcl = with pkgs.rosPackages.${rosDistro}; [
            rcl
            rcl-interfaces
            rclcpp
            rcutils
          ];

          # ros-z-msgs needs message packages
          messages = with pkgs.rosPackages.${rosDistro}; [
            std-msgs
            geometry-msgs
            sensor-msgs
            example-interfaces
            common-interfaces
            rosidl-default-generators
            rosidl-default-runtime
            rosidl-adapter
          ];

          # Additional packages for development/testing
          devExtras = with pkgs.rosPackages.${rosDistro}; [
            ament-cmake-core
            ros-core
            demo-nodes-cpp
            demo-nodes-py
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

        # Create ROS environments for different purposes
        rosEnvDev = pkgs.rosPackages.${rosDistro}.buildEnv {
          paths = rosDeps.rcl ++ rosDeps.messages ++ rosDeps.devExtras;
        };

        rosEnvRcl = pkgs.rosPackages.${rosDistro}.buildEnv {
          paths = rosDeps.rcl;
        };

        rosEnvMsgs = pkgs.rosPackages.${rosDistro}.buildEnv {
          paths = rosDeps.messages;
        };

        # Build dependencies: union of what rcl-z and ros-z-msgs need
        rosEnvBuild = pkgs.rosPackages.${rosDistro}.buildEnv {
          paths = rosDeps.rcl ++ rosDeps.messages;
        };

        # Common build inputs shared across shells
        commonBuildInputs = with pkgs; [
          rustToolchain
          cargo-edit
          cargo-watch
          sccache
          clang
          llvmPackages.libclang
          llvmPackages.bintools
          pkg-config
          nushell
        ];

        # Common environment variables
        commonEnvVars = {
          LIBCLANG_PATH = "${pkgs.llvmPackages.libclang.lib}/lib";
          CLANG_PATH = "${pkgs.llvmPackages.clang}/bin/clang";
          LD_LIBRARY_PATH = pkgs.lib.makeLibraryPath [ pkgs.stdenv.cc.cc.lib ];
          RUST_BACKTRACE = "1";
          RMW_IMPLEMENTATION = "rmw_zenoh_cpp";
          RUSTC_WRAPPER = "${pkgs.sccache}/bin/sccache";
        };

        # Shell hook generator
        mkShellHook =
          { showBanner ? true }:
          ''
            ${pkgs.lib.concatStringsSep "\n" (
              pkgs.lib.mapAttrsToList (name: value: "export ${name}=\"${value}\"") commonEnvVars
            )}

            ${pkgs.lib.optionalString showBanner ''
              echo "🦀 ros-z development environment"
              echo "ROS 2 Distribution: ${rosDistro}"
              echo "RMW Implementation: rmw_zenoh_cpp"
              echo "Rust: $(rustc --version)"
              echo "sccache: enabled"
              echo ""
              echo "Workspace members:"
              echo "  ros-z         - Core (no ROS deps)"
              echo "  rcl-z         - RCL bindings (needs rcl)"
              echo "  ros-z-codegen - Code generation"
              echo "  ros-z-msgs    - Message definitions (needs ROS messages)"
              echo ""
              echo "Available commands:"
              echo "  cargo build -p ros-z           - Build core (no ROS deps)"
              echo "  cargo build -p rcl-z           - Build RCL bindings"
              echo "  cargo build -p ros-z-msgs      - Build messages"
              echo "  cargo build                    - Build all members"
              echo "  cargo test                     - Run tests"
              echo "  cargo check --all-features     - Check all features"
              echo "  cargo clippy                   - Run linter"
              echo "  nu scripts/interop.nu          - Run interop tests"
              echo "  nix fmt                        - Format flake"
              echo "  sccache --show-stats           - Show cache statistics"
            ''}
          '';

        # Read Cargo.toml metadata
        cargoToml = builtins.fromTOML (builtins.readFile ./Cargo.toml);
      in
      {
        # Default development shell - includes everything for all workspace members
        devShells.default = pkgs.mkShell {
          name = "ros-z-dev";

          packages =
            commonBuildInputs
            ++ (with pkgs; [
              clang-tools
              rust-analyzer
              nixfmt-rfc-style
            ])
            ++ [ rosEnvDev ];

          shellHook = mkShellHook { showBanner = true; };

          hardeningDisable = [ "all" ];
        };

        # CI shell - includes only build dependencies
        devShells.ci = pkgs.mkShell {
          name = "ros-z-ci";

          packages = commonBuildInputs ++ [ rosEnvBuild ];

          shellHook = mkShellHook { showBanner = false; };

          hardeningDisable = [ "all" ];
        };

        # Minimal shell - no ROS dependencies (for ros-z core development)
        devShells.minimal = pkgs.mkShell {
          name = "ros-z-minimal";

          packages =
            commonBuildInputs
            ++ (with pkgs; [
              clang-tools
              rust-analyzer
              nixfmt-rfc-style
            ]);

          shellHook = ''
            ${pkgs.lib.concatStringsSep "\n" (
              pkgs.lib.mapAttrsToList (name: value: "export ${name}=\"${value}\"") commonEnvVars
            )}
            echo "🦀 ros-z minimal environment (no ROS dependencies)"
            echo "Rust: $(rustc --version)"
          '';

          hardeningDisable = [ "all" ];
        };

        # Formatter for `nix fmt`
        formatter = pkgs.nixfmt-rfc-style;

        # Multiple package outputs for different workspace members
        packages = {
          # ros-z core - no ROS dependencies
          ros-z = pkgs.rustPlatform.buildRustPackage {
            pname = "ros-z";
            version = cargoToml.workspace.package.version;
            src = ./.;
            buildAndTestSubdir = "ros-z";

            cargoLock = {
              lockFile = ./Cargo.lock;
            };

            nativeBuildInputs = commonBuildInputs;
            # No ROS buildInputs for ros-z core

            inherit (commonEnvVars) LIBCLANG_PATH CLANG_PATH RUSTC_WRAPPER;

            meta = with pkgs.lib; {
              description = "Native Rust ROS 2 implementation using Zenoh - Core";
              homepage = cargoToml.workspace.package.homepage;
              license = licenses.asl20;
              maintainers = [ ];
            };
          };

          # rcl-z - needs RCL packages
          rcl-z = pkgs.rustPlatform.buildRustPackage {
            pname = "rcl-z";
            version = cargoToml.workspace.package.version;
            src = ./.;
            buildAndTestSubdir = "rcl-z";

            cargoLock = {
              lockFile = ./Cargo.lock;
            };

            nativeBuildInputs = commonBuildInputs;
            buildInputs = [ rosEnvRcl ];

            inherit (commonEnvVars) LIBCLANG_PATH CLANG_PATH RUSTC_WRAPPER;

            meta = with pkgs.lib; {
              description = "Native Rust ROS 2 implementation using Zenoh - RCL Bindings";
              homepage = cargoToml.workspace.package.homepage;
              license = licenses.asl20;
              maintainers = [ ];
            };
          };

          # ros-z-msgs - needs message packages
          ros-z-msgs = pkgs.rustPlatform.buildRustPackage {
            pname = "ros-z-msgs";
            version = cargoToml.workspace.package.version;
            src = ./.;
            buildAndTestSubdir = "ros-z-msgs";

            cargoLock = {
              lockFile = ./Cargo.lock;
            };

            nativeBuildInputs = commonBuildInputs;
            buildInputs = [ rosEnvMsgs ];

            inherit (commonEnvVars) LIBCLANG_PATH CLANG_PATH RUSTC_WRAPPER;

            meta = with pkgs.lib; {
              description = "Native Rust ROS 2 implementation using Zenoh - Message Definitions";
              homepage = cargoToml.workspace.package.homepage;
              license = licenses.asl20;
              maintainers = [ ];
            };
          };

          # Default: build everything with all dependencies
          default = pkgs.rustPlatform.buildRustPackage {
            pname = "ros-z";
            version = cargoToml.workspace.package.version;
            src = ./.;

            cargoLock = {
              lockFile = ./Cargo.lock;
            };

            nativeBuildInputs = commonBuildInputs;
            buildInputs = [ rosEnvBuild ];

            inherit (commonEnvVars) LIBCLANG_PATH CLANG_PATH RUSTC_WRAPPER;

            meta = with pkgs.lib; {
              description = cargoToml.workspace.package.description;
              homepage = cargoToml.workspace.package.homepage;
              license = licenses.asl20;
              maintainers = [ ];
            };
          };
        };
      }
    );

  nixConfig = {
    extra-substituters = [ "https://ros.cachix.org" ];
    extra-trusted-public-keys = [ "ros.cachix.org-1:dSyZxI8geDCJrwgvCOHDoAfOm5sV1wCPjBkKL+38Rvo=" ];
  };
}
