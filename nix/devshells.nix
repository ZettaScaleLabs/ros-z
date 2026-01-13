# nix/devshells.nix
{
  pkgs,
  pre-commit-check,
  rosEnv,
  system,
}:
let
  # Rust toolchain & tools
  rustToolchain = pkgs.rust-bin.stable."1.91.0".default.override {
    extensions = [
      "rust-src"
      "rust-analyzer"
    ];
  };

  rustfmtNightly = pkgs.rust-bin.nightly.latest.rustfmt;

  rustfmtNightlyBin = pkgs.writeShellScriptBin "rustfmt" ''
    exec ${rustfmtNightly}/bin/rustfmt "$@"
  '';

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

  devTools = with pkgs; [
    cargo-edit
    cargo-watch
    clang-tools
    rust-analyzer
    nixfmt-rfc-style
    gdb
  ];

  docTools = with pkgs; [
    mdbook
    mdbook-admonish
    mdbook-mermaid
  ];

  testTools = with pkgs; [
    cargo-nextest
  ];

  commonEnvVars = rec {
    LIBCLANG_PATH = "${pkgs.llvmPackages.libclang.lib}/lib";
    CLANG_PATH = "${pkgs.llvmPackages.clang}/bin/clang";
    LD_LIBRARY_PATH = pkgs.lib.makeLibraryPath [
      pkgs.stdenv.cc.cc.lib
      pkgs.llvmPackages.libclang.lib
      pkgs.libffi
    ];
    RUST_BACKTRACE = "1";
    RMW_IMPLEMENTATION = "rmw_zenoh_cpp";
    RUSTC_WRAPPER = "${pkgs.sccache}/bin/sccache";
  };

  exportEnvVars = pkgs.lib.toShellVars commonEnvVars;

  mdbookInstallHook = ''
    mdbook-mermaid install book/ 2>/dev/null || true
    mdbook-admonish install book/ 2>/dev/null || true
  '';

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

  # ROS shells for a given distro
  mkRosShells = rosDistro: {
    default = mkDevShell {
      name = "ros-z-dev-${rosDistro}";
      packages = [
        rustfmtNightlyBin
      ]
      ++ commonBuildInputs
      ++ devTools
      ++ docTools
      ++ testTools
      ++ [ (rosEnv.mkRosEnv rosDistro "dev") ]
      ++ pre-commit-check.enabledPackages;
      extraShellHook = ''
        ${pre-commit-check.shellHook}
        ${mdbookInstallHook}
      '';
      banner = ''
        echo "🦀 ros-z development environment (with ROS)"
        echo "ROS 2 Distribution: ${rosDistro}"
        echo "Rust: $(rustc --version)"
      '';
    };

    ci = mkDevShell {
      name = "ros-z-ci-${rosDistro}";
      packages = commonBuildInputs ++ docTools ++ testTools ++ [ (rosEnv.mkRosEnv rosDistro "testFull") ];
      extraShellHook = mdbookInstallHook;
    };
  };

  allDistroShells = builtins.listToAttrs (
    builtins.map (distro: {
      name = distro;
      value = mkRosShells distro;
    }) rosEnv.distros
  );
in
{
  inherit
    rustToolchain
    rustfmtNightly
    rustfmtNightlyBin
    commonBuildInputs
    devTools
    docTools
    testTools
    mkDevShell
    mdbookInstallHook
    allDistroShells
    ;
}
