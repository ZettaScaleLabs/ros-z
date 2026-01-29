# Reproducible Development with Nix

```admonish warning
This is an advanced topic for users familiar with Nix. If you're new to Nix, you can safely skip this chapter and use the standard [Building](./building.md) instructions instead.
```

ros-z provides Nix flakes for reproducible development environments with all dependencies pre-configured.

## Why Use Nix?

**Reproducible Builds:** Nix ensures every developer and CI system uses the exact same dependencies, eliminating "works on my machine" issues. All dependencies—from compilers to ROS 2 packages—are pinned to specific versions and cached immutably.

**Uniform CI/CD:** The same Nix configuration that runs locally is used in continuous integration, ensuring build consistency across development, testing, and deployment environments. No more divergence between local builds and CI failures.

**Zero Setup:** New team members can start developing with a single `nix develop` command—no manual ROS installation, no dependency hunting, no environment configuration.

## Available Environments

```bash
# Default: ROS 2 Jazzy with full tooling
nix develop

# Specific ROS distros
nix develop .#ros-jazzy      # ROS 2 Jazzy
nix develop .#ros-rolling    # ROS 2 Rolling

# Pure Rust (no ROS)
nix develop .#pureRust       # Minimal Rust toolchain only
```

## Use Cases

| Use Case | Environment | Benefit |
| --- | --- | --- |
| **Team Development** | All developers | Everyone has identical toolchains and dependencies |
| **CI/CD Pipelines** | GitHub Actions, GitLab CI | Same environment locally and in automation |
| **Cross-Platform** | Linux, macOS, WSL | Consistent builds regardless of host OS |
| **Multiple ROS Versions** | Switch environments easily | Test against different ROS distros without conflicts |

```admonish tip
Use Nix for consistent development environments across team members and CI/CD pipelines. The reproducibility guarantees catch integration issues early.
```

## Learning More

- **[Nix Official Guide](https://nixos.org/manual/nix/stable/)** - Introduction to Nix
- **[Nix Flakes](https://nixos.wiki/wiki/Flakes)** - Understanding flakes
- **[ros-z flake.nix](https://github.com/ZettaScaleLabs/ros-z/blob/main/flake.nix)** - Our Nix configuration
