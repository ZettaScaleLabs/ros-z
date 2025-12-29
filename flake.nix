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
        pkgs = import nixpkgs {
          inherit system;
          overlays = [
            nix-ros-overlay.overlays.default
            rust-overlay.overlays.default
          ];
        };

        rosEnv = import ./nix/ros-env.nix { inherit pkgs; };

        devshells = import ./nix/devshells.nix {
          inherit
            pkgs
            pre-commit-check
            rosEnv
            system
            ;
        };

        pre-commit-check = import ./nix/pre-commit.nix {
          inherit pkgs git-hooks system;
          rustfmtNightly = devshells.rustfmtNightly;
        };

        inherit (devshells)
          commonBuildInputs
          devTools
          docTools
          testTools
          mkDevShell
          rustfmtNightlyBin
          mdbookInstallHook
          allDistroShells
          ;

        distros = rosEnv.distros;

        makeDistroShells =
          suffix:
          builtins.listToAttrs (
            builtins.map (distro: {
              name = "ros-${distro}${suffix}";
              value = allDistroShells.${distro}.${if suffix == "" then "default" else "ci"};
            }) distros
          );
      in
      {
        checks = {
          inherit pre-commit-check;
        };

        devShells = {
          default = allDistroShells.${builtins.head distros}.default;

          pureRust = mkDevShell {
            name = "ros-z-pure-rust";
            packages = [
              rustfmtNightlyBin
            ]
            ++ commonBuildInputs
            ++ devTools
            ++ docTools
            ++ testTools
            ++ pre-commit-check.enabledPackages;
            extraShellHook = ''
              ${pre-commit-check.shellHook}
              ${mdbookInstallHook}
            '';
            banner = ''
              echo "🦀 ros-z development environment (pure Rust)"
              echo "Rust: $(rustc --version)"
            '';
          };

          pureRust-ci = mkDevShell {
            name = "ros-z-ci-pure-rust";
            packages = commonBuildInputs ++ docTools ++ testTools;
            extraShellHook = mdbookInstallHook;
          };
        }
        // (makeDistroShells "")
        // (makeDistroShells "-ci");

        formatter = pkgs.nixfmt-rfc-style;
      }
    );

  nixConfig = {
    extra-substituters = [ "https://ros.cachix.org" ];
    extra-trusted-public-keys = [ "ros.cachix.org-1:dSyZxI8geDCJrwgvCOHDoAfOm5sV1wCPjBkKL+38Rvo=" ];
  };
}
