{
  description = "ros-z: Native Rust ROS 2 implementation using Zenoh";

  inputs = {
    nixpkgs.follows = "nix-ros-overlay/nixpkgs";
    nix-ros-overlay.url = "github:lopsided98/nix-ros-overlay";
    rust-overlay.url = "github:oxalica/rust-overlay";
    nix-github-actions.url = "github:nix-community/nix-github-actions";
    nix-github-actions.inputs.nixpkgs.follows = "nixpkgs";
  };

  outputs =
    {
      self,
      nixpkgs,
      nix-ros-overlay,
      rust-overlay,
      nix-github-actions,
    }:
    let
      systemOutputs = nix-ros-overlay.inputs.flake-utils.lib.eachDefaultSystem (
        system:
        let
          inherit (pkgs.lib)
            concatStringsSep
            mapAttrsToList
            ;

          # Supported ROS 2 distributions
          distros = [
            "jazzy" # Default - ROS 2 LTS
            "rolling" # Latest development
          ];

          # Rust toolchain version
          rustVersion = "1.91.0";

          pkgs = import nixpkgs {
            inherit system;
            overlays = [
              nix-ros-overlay.overlays.default
              rust-overlay.overlays.default
            ];
          };

          # Rust toolchain with required extensions
          rustToolchain = pkgs.rust-bin.stable.${rustVersion}.default.override {
            extensions = [
              "rust-src"
              "rust-analyzer"
            ];
          };

          # Create ROS environment for a specific distribution
          # Returns: { dev, rcl, msgs, build } environments
          mkRosEnv =
            rosDistro:
            let
              inherit (pkgs.rosPackages.${rosDistro}) buildEnv;

              # Core RCL dependencies for ROS 2 client library
              rclDeps = with pkgs.rosPackages.${rosDistro}; [
                rcl
                rcl-interfaces
                rclcpp
                rcutils
              ];

              # Message packages for code generation
              msgDeps = with pkgs.rosPackages.${rosDistro}; [
                std-msgs
                geometry-msgs
                sensor-msgs
                example-interfaces
                common-interfaces
                rosidl-default-generators
                rosidl-default-runtime
                rosidl-adapter
              ];

              # Development and testing extras
              devDeps = with pkgs.rosPackages.${rosDistro}; [
                ament-cmake-core
                ros-core
                demo-nodes-cpp
                demo-nodes-py
                rclpy
                rmw
                rmw-implementation
                rmw-zenoh-cpp # Required for interop tests
                ament-cmake
                ament-cmake-gtest
                ament-lint-auto
                ament-lint-common
                launch
                launch-testing
                ros2cli
              ];
            in
            {
              dev = buildEnv { paths = rclDeps ++ msgDeps ++ devDeps; };
              rcl = buildEnv { paths = rclDeps; };
              msgs = buildEnv { paths = msgDeps; };
              build = buildEnv { paths = rclDeps ++ msgDeps; };
            };

          # Common build tools for all environments
          commonBuildInputs = with pkgs; [
            rustToolchain
            sccache # Rust compilation cache
            clang # Required for bindgen
            llvmPackages.libclang
            llvmPackages.bintools
            pkg-config
            nushell # Shell scripting
            protobuf # Protocol buffers support
            markdownlint-cli # Documentation linting
          ];

          # Additional development tools
          devTools = with pkgs; [
            cargo-edit # cargo add/rm/upgrade
            cargo-watch # Auto-rebuild on changes
            clang-tools # clangd LSP
            rust-analyzer # Rust LSP
            nixfmt-rfc-style # Nix formatter
            gdb # Debugger
          ];

          # Common environment variables for Rust/C++ interop
          commonEnvVars = {
            LIBCLANG_PATH = "${pkgs.llvmPackages.libclang.lib}/lib";
            CLANG_PATH = "${pkgs.llvmPackages.clang}/bin/clang";
            LD_LIBRARY_PATH = pkgs.lib.makeLibraryPath [ pkgs.stdenv.cc.cc.lib ];
            RUST_BACKTRACE = "1";
            RMW_IMPLEMENTATION = "rmw_zenoh_cpp";
            RUSTC_WRAPPER = "${pkgs.sccache}/bin/sccache";
          };

          # Export environment variables as shell commands
          exportEnvVars = concatStringsSep "\n" (
            mapAttrsToList (name: value: "export ${name}=\"${value}\"") commonEnvVars
          );

          # Create a development shell with specified configuration
          mkDevShell =
            {
              name,
              packages,
              banner ? "",
            }:
            pkgs.mkShell {
              inherit name packages;
              shellHook = exportEnvVars + (if banner != "" then "\n${banner}" else "");
              hardeningDisable = [ "all" ]; # Disable hardening for ROS compatibility
            };

          # Create shells for a specific ROS distribution
          mkRosShells =
            rosDistro:
            let
              rosEnv = mkRosEnv rosDistro;
              distroName = "${rosDistro}";
            in
            {
              # Full development environment
              default = mkDevShell {
                name = "ros-z-dev-${distroName}";
                packages = commonBuildInputs ++ devTools ++ [ rosEnv.dev ];
                banner = ''
                  echo "🦀 ros-z development environment"
                  echo "ROS 2 Distribution: ${distroName}"
                  echo "Rust: $(rustc --version)"
                '';
              };

              # Minimal CI environment (no dev tools)
              ci = mkDevShell {
                name = "ros-z-ci-${distroName}";
                packages = commonBuildInputs ++ [ rosEnv.build ];
              };
            };

          # Generate shells for all supported distributions
          allDistroShells = builtins.listToAttrs (
            builtins.map (distro: {
              name = distro;
              value = mkRosShells distro;
            }) distros
          );

          # CI checks - imported from ci.nix module
          ciChecks = import ./ci.nix {
            inherit
              pkgs
              self
              commonBuildInputs
              mkRosEnv
              exportEnvVars
              ;
          };
        in
        {
          # Development shells
          devShells = {
            # Default: Jazzy with full dev tools
            default = allDistroShells.${builtins.head distros}.default;

            # No ROS dependencies (for core library development)
            noRos = mkDevShell {
              name = "ros-z-no-ros";
              packages = commonBuildInputs ++ devTools;
              banner = ''
                echo "🦀 ros-z development environment (without ROS)"
                echo "Rust: $(rustc --version)"
              '';
            };

            # No ROS, minimal (for CI)
            noRos-ci = mkDevShell {
              name = "ros-z-ci-no-ros";
              packages = commonBuildInputs;
            };
          }
          # Add per-distro dev shells (jazzy, rolling)
          // (builtins.listToAttrs (
            builtins.map (distro: {
              name = distro;
              value = allDistroShells.${distro}.default;
            }) distros
          ))
          # Add per-distro CI shells (jazzy-ci, rolling-ci)
          // (builtins.listToAttrs (
            builtins.map (distro: {
              name = "${distro}-ci";
              value = allDistroShells.${distro}.ci;
            }) distros
          ));

          # Formatter for nix files
          formatter = pkgs.nixfmt-rfc-style;

          # CI checks (sandboxed with vendored cargo)
          checks = ciChecks;
        }
      );
    in
    systemOutputs
    // {
      # GitHub Actions matrix generation
      githubActions = nix-github-actions.lib.mkGithubMatrix {
        checks = nixpkgs.lib.getAttrs [ "x86_64-linux" ] systemOutputs.checks;
      };
    };

  # Nix configuration for binary caches
  nixConfig = {
    extra-substituters = [ "https://ros.cachix.org" ];
    extra-trusted-public-keys = [ "ros.cachix.org-1:dSyZxI8geDCJrwgvCOHDoAfOm5sV1wCPjBkKL+38Rvo=" ];
  };
}
