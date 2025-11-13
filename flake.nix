{
  description = "ros-z: Native Rust ROS 2 implementation using Zenoh";

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
            rust-overlay.overlays.default
          ];
        };

        rustToolchain = pkgs.rust-bin.stable."1.91.0".default.override {
          extensions = [
            "rust-src"
            "rust-analyzer"
          ];
        };

        rosDistro = "jazzy";

        # ROS package dependencies organized by purpose
        rosDeps = {
          rcl = with pkgs.rosPackages.${rosDistro}; [
            rcl
            rcl-interfaces
            rclcpp
            rcutils
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
          ];

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

        # ROS environments for different build contexts
        rosEnv = {
          dev = pkgs.rosPackages.${rosDistro}.buildEnv {
            paths = rosDeps.rcl ++ rosDeps.messages ++ rosDeps.devExtras;
          };
          rcl = pkgs.rosPackages.${rosDistro}.buildEnv {
            paths = rosDeps.rcl;
          };
          msgs = pkgs.rosPackages.${rosDistro}.buildEnv {
            paths = rosDeps.messages;
          };
          build = pkgs.rosPackages.${rosDistro}.buildEnv {
            paths = rosDeps.rcl ++ rosDeps.messages;
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
        ];

        # Development tools
        devTools = with pkgs; [
          cargo-edit
          cargo-watch
          clang-tools
          rust-analyzer
          nixfmt-rfc-style
          gdb
        ];

        # Environment variables for Rust/C++ interop
        commonEnvVars = {
          LIBCLANG_PATH = "${pkgs.llvmPackages.libclang.lib}/lib";
          CLANG_PATH = "${pkgs.llvmPackages.clang}/bin/clang";
          LD_LIBRARY_PATH = pkgs.lib.makeLibraryPath [ pkgs.stdenv.cc.cc.lib ];
          RUST_BACKTRACE = "1";
          RMW_IMPLEMENTATION = "rmw_zenoh_cpp";
          RUSTC_WRAPPER = "${pkgs.sccache}/bin/sccache";
        };

        # Export environment variables as shell commands
        exportEnvVars = pkgs.lib.concatStringsSep "\n" (
          pkgs.lib.mapAttrsToList (name: value: "export ${name}=\"${value}\"") commonEnvVars
        );

        # Banner text for development shell
        bannerText = ''
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
        '';

        # Generate shell hook with optional banner
        mkShellHook =
          { showBanner ? true }:
          exportEnvVars + pkgs.lib.optionalString showBanner ("\n" + bannerText);

        # Base shell configuration factory
        mkDevShell =
          {
            name,
            packages,
            showBanner ? true,
          }:
          pkgs.mkShell {
            inherit name packages;
            shellHook = mkShellHook { inherit showBanner; };
            hardeningDisable = [ "all" ];
          };

        # Cargo metadata
        cargoToml = builtins.fromTOML (builtins.readFile ./Cargo.toml);

        # Common package attributes factory
        mkRustPackage =
          {
            pname,
            buildAndTestSubdir ? null,
            rosInputs ? [ ],
            description,
          }:
          pkgs.rustPlatform.buildRustPackage {
            inherit pname;
            version = cargoToml.workspace.package.version;
            src = ./.;
            inherit buildAndTestSubdir;

            cargoLock = {
              lockFile = ./Cargo.lock;
              outputHashes = {
                "cdr-encoding-0.10.2" = "sha256-bpo8Fu3Qp5TapzFFAvyRJdSiO50G3YBBTSJNV/cNa4Y=";
              };
            };

            nativeBuildInputs = commonBuildInputs;
            buildInputs = rosInputs;

            inherit (commonEnvVars) LIBCLANG_PATH CLANG_PATH RUSTC_WRAPPER;

            meta = with pkgs.lib; {
              inherit description;
              homepage = cargoToml.workspace.package.homepage;
              license = licenses.asl20;
              maintainers = [ ];
            };
          };
      in
      {
        # Development shells
        devShells = {
          default = mkDevShell {
            name = "ros-z-dev";
            packages = commonBuildInputs ++ devTools ++ [ rosEnv.dev ];
          };

          noBanner = mkDevShell {
            name = "ros-z-dev";
            packages = commonBuildInputs ++ devTools ++ [ rosEnv.dev ];
            showBanner = false;
          };

          ci = mkDevShell {
            name = "ros-z-ci";
            packages = commonBuildInputs ++ [ rosEnv.build ];
            showBanner = false;
          };

          minimal = pkgs.mkShell {
            name = "ros-z-minimal";
            packages = commonBuildInputs ++ devTools;
            shellHook = ''
              ${exportEnvVars}
              echo "🦀 ros-z minimal environment (no ROS dependencies)"
              echo "Rust: $(rustc --version)"
            '';
            hardeningDisable = [ "all" ];
          };
        };

        # Package outputs
        packages = {
          ros-z = mkRustPackage {
            pname = "ros-z";
            buildAndTestSubdir = "ros-z";
            description = "Native Rust ROS 2 implementation using Zenoh - Core";
          };

          rcl-z = mkRustPackage {
            pname = "rcl-z";
            buildAndTestSubdir = "rcl-z";
            rosInputs = [ rosEnv.rcl ];
            description = "Native Rust ROS 2 implementation using Zenoh - RCL Bindings";
          };

          ros-z-msgs = mkRustPackage {
            pname = "ros-z-msgs";
            buildAndTestSubdir = "ros-z-msgs";
            rosInputs = [ rosEnv.msgs ];
            description = "Native Rust ROS 2 implementation using Zenoh - Message Definitions";
          };

          default = mkRustPackage {
            pname = "ros-z";
            rosInputs = [ rosEnv.build ];
            description = cargoToml.workspace.package.description;
          };
        };

        formatter = pkgs.nixfmt-rfc-style;
      }
    );

  nixConfig = {
    extra-substituters = [ "https://ros.cachix.org" ];
    extra-trusted-public-keys = [ "ros.cachix.org-1:dSyZxI8geDCJrwgvCOHDoAfOm5sV1wCPjBkKL+38Rvo=" ];
  };
}
