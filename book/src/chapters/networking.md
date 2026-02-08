# Networking

**Configure ros-z's Zenoh transport layer for optimal performance in your deployment environment.** ros-z uses router-based architecture by default, matching ROS 2's official `rmw_zenoh_cpp` middleware for production-ready scalability.

## Router-Based Architecture

ros-z uses a centralized Zenoh router for node discovery and communication, providing:

| Benefit | Description |
|---------|-------------|
| **Scalability** | Centralized discovery handles large deployments efficiently |
| **Lower Network Overhead** | TCP-based discovery instead of multicast broadcasts |
| **ROS 2 Compatibility** | Matches `rmw_zenoh_cpp` behavior for seamless interoperability |
| **Production Ready** | Battle-tested configuration used in real robotics systems |

## Quick Start

The default ros-z configuration connects to a Zenoh router on `tcp/localhost:7447`:

```rust,ignore
use ros_z::context::ZContextBuilder;
use ros_z::Builder;

// Uses default ROS session config (connects to tcp/localhost:7447)
let ctx = ZContextBuilder::default().build()?;
let node = ctx.create_node("my_node").build()?;
```

```admonish success
That's it! The default configuration automatically connects to the router. Now you just need to run one.
```

## Running the Zenoh Router

ros-z applications require a Zenoh router to be running. There are several ways to get one - choose based on your environment and requirements.

### Quick Comparison

| Method | Best For | Requires | Installation Time |
|--------|----------|----------|-------------------|
| [Cargo Install](#method-1-cargo-install) | Rust developers | Rust toolchain | 2-5 minutes |
| [Pre-built Binary](#method-2-pre-built-binary) | Quick setup, no Rust | None | < 1 minute |
| [Docker](#method-3-docker) | Containers, CI/CD | Docker | < 1 minute |
| [Package Manager](#method-4-package-manager-apt-brew) | System-wide install | apt/brew/etc | < 1 minute |
| [ros-z Example](#method-5-ros-z-example-router) | ros-z repo developers | ros-z repository | 30 seconds |
| [ROS 2 rmw_zenoh](#method-6-ros-2-rmw_zenoh) | ROS 2 interop testing | ROS 2 installed | Already installed |

---

### Method 1: Cargo Install

**Recommended for Rust developers building standalone projects.**

Install the official Zenoh router using Cargo:

```bash
cargo install zenohd
```

Run the router:

```bash
zenohd
```

**Pros:**

- Always up-to-date with latest Zenoh
- Builds optimized for your system
- Easy to update: `cargo install zenohd --force`

**Cons:**

- Requires Rust toolchain
- Takes 2-5 minutes to compile

---

### Method 2: Pre-built Binary

**Fastest way to get started without Rust installed.**

Download the latest release for your platform:

**Linux (x86_64):**

```bash
wget https://github.com/eclipse-zenoh/zenoh/releases/latest/download/zenoh-x86_64-unknown-linux-gnu.zip
unzip zenoh-x86_64-unknown-linux-gnu.zip
chmod +x zenohd
./zenohd
```

**macOS (Apple Silicon):**

```bash
wget https://github.com/eclipse-zenoh/zenoh/releases/latest/download/zenoh-aarch64-apple-darwin.zip
unzip zenoh-aarch64-apple-darwin.zip
chmod +x zenohd
./zenohd
```

**macOS (Intel):**

```bash
wget https://github.com/eclipse-zenoh/zenoh/releases/latest/download/zenoh-x86_64-apple-darwin.zip
unzip zenoh-x86_64-apple-darwin.zip
chmod +x zenohd
./zenohd
```

**Windows (PowerShell):**

```powershell
Invoke-WebRequest -Uri "https://github.com/eclipse-zenoh/zenoh/releases/latest/download/zenoh-x86_64-pc-windows-msvc.zip" -OutFile "zenoh.zip"
Expand-Archive zenoh.zip
.\zenoh\zenohd.exe
```

**Pros:**

- No build tools required
- Instant startup
- Portable - can run from any directory

**Cons:**

- Manual download and extraction
- Need to track updates yourself

**More info:** <https://zenoh.io/docs/getting-started/installation/>

---

### Method 3: Docker

**Perfect for containerized deployments and CI/CD pipelines.**

Pull and run the official Zenoh router image:

```bash
docker run --init --net host eclipse/zenoh:latest
```

**For production with persistent config:**

```bash
docker run -d \
  --name zenoh-router \
  --net host \
  -v /path/to/config:/zenoh/config \
  eclipse/zenoh:latest \
  --config /zenoh/config/zenoh.json5
```

**Pros:**

- Isolated from host system
- Easy to deploy and scale
- Works great in Kubernetes/Docker Compose
- Consistent across environments

**Cons:**

- Requires Docker installed
- Network setup can be tricky (use `--net host` for simplicity)

**Docker Hub:** <https://hub.docker.com/r/eclipse/zenoh>

---

### Method 4: Package Manager (apt, brew)

**Best for system-wide installation on Linux/macOS.**

**Ubuntu/Debian (via apt):**

```bash
echo "deb [trusted=yes] https://download.eclipse.org/zenoh/debian-repo/ /" | sudo tee /etc/apt/sources.list.d/zenoh.list
sudo apt update
sudo apt install zenoh
```

Run as a service:

```bash
sudo systemctl enable zenoh
sudo systemctl start zenoh
```

Or run manually:

```bash
zenohd
```

**macOS (via Homebrew):**

```bash
brew tap eclipse-zenoh/homebrew-zenoh
brew install zenoh
zenohd
```

**Arch Linux (via AUR):**

```bash
yay -S zenoh
zenohd
```

**Pros:**

- System-wide installation
- Easy updates via package manager
- Can run as systemd service (Linux)
- Integrates with OS security/firewall settings

**Cons:**

- May not have the absolute latest version
- Requires sudo/admin privileges

---

### Method 5: ros-z Example Router

**Only available when working in the ros-z repository - perfect for quick development/testing.**

If you've cloned the ros-z repository:

```bash
cd /path/to/ros-z
cargo run --example zenoh_router
```

This runs a pre-configured router that matches ros-z defaults exactly.

**Pros:**

- No installation needed
- Already configured for ros-z
- Useful for debugging ros-z itself

**Cons:**

- Only available in the ros-z repository
- Not suitable for standalone projects
- Slower startup (rebuilds if code changes)

```admonish warning
This method is for ros-z repository development only. If you're building your own project with ros-z as a dependency, use one of the other methods instead.
```

---

### Method 6: ROS 2 rmw_zenoh

**Use this if you have ROS 2 installed and want to test interoperability with ROS 2 nodes.**

If you have ROS 2 Jazzy or newer with the Zenoh middleware:

```bash
ros2 run rmw_zenoh_cpp rmw_zenohd
```

**Pros:**

- Already installed with ROS 2 Jazzy+
- Guaranteed compatibility with ROS 2 nodes
- Can interoperate ros-z nodes with C++/Python ROS 2 nodes

**Cons:**

- Requires full ROS 2 installation
- Overkill if you only want to use ros-z

```admonish tip
This is excellent for testing interoperability - run ros-z nodes alongside standard ROS 2 nodes using the same router.
```

---

## Verifying the Router is Running

After starting a router with any method above, verify it's working:

**Check the router is listening on port 7447:**

```bash
# Linux/macOS
netstat -an | grep 7447

# Or use lsof
lsof -i :7447
```

**Test with a ros-z application:**

```bash
# In another terminal, try running a ros-z node
# If it connects successfully, the router is working
```

You should see log output from the router showing connections when your ros-z nodes start.

## Next Steps

Choose the configuration approach that fits your needs:

- **[Configuration Options](./config_options.md)** - Six ways to configure Zenoh (from simple to complex)
- **[Advanced Configuration](./config_advanced.md)** - Generate config files, run routers, configuration reference
- **[Troubleshooting](./troubleshooting.md)** - Solutions to connectivity issues

**Ready to optimize your deployment? Experiment with different configurations and measure performance impact.**
