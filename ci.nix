# CI checks module
# This module defines all CI checks for ros-z project
{
  pkgs,
  self,
  commonBuildInputs,
  mkRosEnv,
  exportEnvVars,
}:
let
  # Vendor Cargo dependencies for sandboxed builds
  cargoVendorDir = pkgs.rustPlatform.importCargoLock {
    lockFile = ./Cargo.lock;
    outputHashes = {
      "cdr-encoding-0.10.2" = "sha256-bpo8Fu3Qp5TapzFFAvyRJdSiO50G3YBBTSJNV/cNa4Y=";
      "roslibrust-0.16.0" = "sha256-qi4h1ksC/iLwK1uiUs6LU9CX3RDYVOd6E4SRdUAbqZo=";
    };
  };

  # Fetch roslibrust git repo for assets directory (with submodules for ros2_common_interfaces)
  roslibrustSrc = pkgs.fetchgit {
    url = "https://github.com/YuanYuYuan/roslibrust";
    rev = "f08547babb04c3b19a77af9a55a10b8148908e55"; # dev/ros-z branch
    hash = "sha256-qi4h1ksC/iLwK1uiUs6LU9CX3RDYVOd6E4SRdUAbqZo=";
    fetchSubmodules = true;
  };
in
let
  # Create a cargo check that runs in sandbox with vendored dependencies
  mkCargoCheck =
    {
      name,
      packages,
      rosEnv ? null,
      script,
      extraAttrs ? { },
    }:
    pkgs.stdenv.mkDerivation (
      {
        name = "check-${name}";
        src = self;
        nativeBuildInputs = packages ++ [ pkgs.nixfmt-rfc-style ]; # For format checks
        buildInputs = pkgs.lib.optional (rosEnv != null) rosEnv;

        # Disable ROS setup hooks that interfere with Cargo builds
        dontUseCmakeConfigure = true;
        dontUseColconBuildSetup = true;

        buildPhase = ''
          # Export environment variables
          ${exportEnvVars}

          # Disable sccache in sandbox (no HOME directory)
          unset RUSTC_WRAPPER

          # Setup ROS environment if provided
          ${pkgs.lib.optionalString (rosEnv != null) ''
            export AMENT_PREFIX_PATH="${rosEnv}"
            export CMAKE_PREFIX_PATH="${rosEnv}"
          ''}

          # Setup vendored cargo dependencies
          export CARGO_HOME=$(pwd)/.cargo
          mkdir -p .cargo
          cat > .cargo/config.toml <<EOF
          [source.crates-io]
          replace-with = "vendored-sources"

          [source."https://github.com/YuanYuYuan/cdr-encoding"]
          git = "https://github.com/YuanYuYuan/cdr-encoding"
          branch = "feat/byte-buf"
          replace-with = "vendored-sources"

          [source."https://github.com/YuanYuYuan/roslibrust"]
          git = "https://github.com/YuanYuYuan/roslibrust"
          branch = "dev/ros-z"
          replace-with = "vendored-sources"

          [source.vendored-sources]
          directory = "${cargoVendorDir}"
          EOF

          # Point build.rs to roslibrust assets from fetched git repo
          export ROSLIBRUST_ASSETS_DIR="${roslibrustSrc}/assets"

          # Run checks
          set -euo pipefail
          ${script}
        '';

        installPhase = ''
          touch $out
        '';

        # Disable hardening for ROS compatibility
        hardeningDisable = [ "all" ];
      }
      // extraAttrs
    );
in
let
  # Formatting check - runs first and other checks depend on this
  formatting = mkCargoCheck {
    name = "formatting";
    packages = commonBuildInputs;
    script = ''
      echo "=== Nix Formatting ==="
      nixfmt --check flake.nix

      echo "=== Markdown Linting ==="
      markdownlint '**/*.md'

      echo "=== Rust Formatting ==="
      cargo fmt --all -- --check
    '';
  };

  # ROS-independent checks (linting, build, test)
  no-ros = mkCargoCheck {
    name = "no-ros";
    packages = commonBuildInputs;
    script = ''
      # This check depends on formatting passing first
      # Reference the formatting check to create a build-time dependency
      echo "Prerequisites: ${formatting}"

      echo "=== Build & Test ==="
      cargo build --workspace --lib --bins --exclude rcl-z --exclude protobuf_demo
      cargo test --workspace --lib --bins --exclude rcl-z --exclude protobuf_demo

      echo "=== Lint ==="
      cargo clippy --workspace --lib --bins --exclude rcl-z -- -D warnings

      echo "=== Message Package Checks ==="
      cargo check -p ros-z-msgs
      cargo check -p ros-z-msgs --features bundled_msgs
      cargo check -p ros-z-msgs --features common_interfaces

      echo "=== Individual Message Packages ==="
      cargo build -p ros-z-msgs --no-default-features --features std_msgs
      cargo build -p ros-z-msgs --no-default-features --features geometry_msgs
      cargo build -p ros-z-msgs --no-default-features --features sensor_msgs
      cargo build -p ros-z-msgs --no-default-features --features nav_msgs
    '';
  };

  # ROS-dependent checks for Jazzy distribution
  with-ros-jazzy = mkCargoCheck {
    name = "with-ros-jazzy";
    packages = commonBuildInputs;
    rosEnv = (mkRosEnv "jazzy").build;
    script = ''
      # This check depends on formatting passing first
      # Reference the formatting check to create a build-time dependency
      echo "Prerequisites: ${formatting}"

      echo "=== RCL Bindings ==="
      cargo check -p rcl-z

      echo "=== Message Packages ==="
      cargo check -p ros-z-msgs
      cargo check -p ros-z-msgs --features external_msgs
      cargo check -p ros-z-msgs --features example_interfaces
      cargo check -p ros-z-msgs --features all_msgs

      echo "=== Feature Checks ==="
      cargo check -p ros-z -p ros-z-msgs --features ros-z/protobuf,ros-z-msgs/protobuf
      cargo check -p ros-z --features rcl-z

      echo "=== Build Examples ==="
      cargo build -p ros-z-msgs
      cargo build --examples
      cargo build --example z_srvcli --features external_msgs
      cargo build -p protobuf_demo
    '';
  };
in
{
  # Export all checks
  inherit formatting no-ros with-ros-jazzy;
}
