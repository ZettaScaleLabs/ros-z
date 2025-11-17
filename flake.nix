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
          in
          {
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

        # Base shell configuration factory
        mkDevShell =
          {
            name,
            packages,
            banner ? "",
          }:
          pkgs.mkShell {
            inherit name packages;
            shellHook = exportEnvVars + (if banner != "" then "\n${banner}" else "");
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
              packages = commonBuildInputs ++ devTools ++ [ rosEnv.dev ];
              banner = ''
                echo "🦀 ros-z development environment (with ROS)"
                echo "ROS 2 Distribution: ${rosDistro}"
                echo "Rust: $(rustc --version)"
              '';
            };

            ci = mkDevShell {
              name = "ros-z-ci-${rosDistro}";
              packages = commonBuildInputs ++ [ rosEnv.build ];
            };
          };

        # Cargo metadata
        cargoToml = builtins.fromTOML (builtins.readFile ./Cargo.toml);

        # Helper to create packages for a specific ROS distro
        mkRosPackages =
          rosDistro:
          let
            rosEnv = mkRosEnv rosDistro;
          in
          {
            "ros-z-${rosDistro}" = mkRustPackage {
              pname = "ros-z";
              buildAndTestSubdir = "ros-z";
              description = "Native Rust ROS 2 implementation using Zenoh - Core";
            };

            "rcl-z-${rosDistro}" = mkRustPackage {
              pname = "rcl-z";
              buildAndTestSubdir = "rcl-z";
              rosInputs = [ rosEnv.rcl ];
              description = "Native Rust ROS 2 implementation using Zenoh - RCL Bindings (${rosDistro})";
            };

            "ros-z-msgs-${rosDistro}" = mkRustPackage {
              pname = "ros-z-msgs";
              buildAndTestSubdir = "ros-z-msgs";
              rosInputs = [ rosEnv.msgs ];
              description = "Native Rust ROS 2 implementation using Zenoh - Message Definitions (${rosDistro})";
            };

            "ros-z-full-${rosDistro}" = mkRustPackage {
              pname = "ros-z";
              rosInputs = [ rosEnv.build ];
              description = cargoToml.workspace.package.description;
            };
          };

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

        # Generate shells for all distros
        allDistroShells = builtins.listToAttrs (
          builtins.map (distro: {
            name = distro;
            value = mkRosShells distro;
          }) distros
        );
      in
      {
        # Development shells
        devShells = {
          # Default: first distro in the list with ROS
          default = allDistroShells.${builtins.head distros}.default;

          # Without ROS
          noRos = mkDevShell {
            name = "ros-z-no-ros";
            packages = commonBuildInputs ++ devTools;
            banner = ''
              echo "🦀 ros-z development environment (without ROS)"
              echo "Rust: $(rustc --version)"
            '';
          };

          # CI without ROS
          noRos-ci = mkDevShell {
            name = "ros-z-ci-no-ros";
            packages = commonBuildInputs;
          };
        }
        # Add per-distro dev shells (jazzy, rolling, ...)
        // (builtins.listToAttrs (
          builtins.map (distro: {
            name = distro;
            value = allDistroShells.${distro}.default;
          }) distros
        ))
        # Add per-distro CI shells (jazzy-ci, rolling-ci, ...)
        // (builtins.listToAttrs (
          builtins.map (distro: {
            name = "${distro}-ci";
            value = allDistroShells.${distro}.ci;
          }) distros
        ));

        # Package outputs
        packages =
          let
            basePackage = mkRustPackage {
              pname = "ros-z";
              buildAndTestSubdir = "ros-z";
              description = "Native Rust ROS 2 implementation using Zenoh - Core";
            };

            # Generate packages for all distros
            allDistroPackages = builtins.foldl' (acc: distro: acc // (mkRosPackages distro)) { } distros;
          in
          {
            ros-z = basePackage;
            default = basePackage;
          }
          // allDistroPackages;

        formatter = pkgs.nixfmt-rfc-style;
      }
    );

  nixConfig = {
    extra-substituters = [ "https://ros.cachix.org" ];
    extra-trusted-public-keys = [ "ros.cachix.org-1:dSyZxI8geDCJrwgvCOHDoAfOm5sV1wCPjBkKL+38Rvo=" ];
  };
}
