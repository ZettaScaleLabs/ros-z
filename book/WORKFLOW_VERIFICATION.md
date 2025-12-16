# Workflow Verification Guide

This document verifies that the complete workflow works as intended.

## Current Structure (Optimal)

```
ros-z/                          # Workspace root
├── Cargo.toml                  # Workspace definition
├── ros-z/                      # Library crate
│   ├── Cargo.toml             # Has all deps: clap, tokio, zenoh
│   ├── src/                   # Library code
│   └── examples/              # Examples (standard location)
│       ├── z_pubsub.rs
│       ├── demo_nodes/
│       │   ├── talker.rs
│       │   └── listener.rs
│       └── ...
├── ros-z-msgs/
├── book/
│   ├── book.toml
│   └── src/
│       └── chapters/
│           └── *.md           # Use {{#include ../../../ros-z/examples/...}}
└── test-docs.sh
```

## Workflow Verification

### Step 1: Examples Have Dependencies

Check `ros-z/Cargo.toml`:

```toml
[dependencies]
clap = { workspace = true, features = ["derive"] }  # ✅ For CLI args
tokio = { workspace = true, features = ["rt-multi-thread"] }  # ✅ For async
zenoh = { workspace = true }  # ✅ For communication
# ... all other dependencies

[dev-dependencies]
ros-z-msgs = { path = "../ros-z-msgs" }  # ✅ For message types
```

**Result:** Examples have full access to all dependencies.

### Step 2: Examples Can Be Run

```bash
# Build an example
cargo build --example demo_nodes_talker

# Run an example with CLI args (clap works!)
cargo run --example demo_nodes_talker -- --help
cargo run --example demo_nodes_talker -- --topic /test --period 0.5

# Run service examples (needs external_msgs feature)
cargo run --example demo_nodes_add_two_ints_client --features external_msgs -- --a 10 --b 20
```

**Verified:** ✅ Examples compile and run correctly.

### Step 3: mdBook Can Test Examples

```bash
# Build the library
cargo build

# Test all code blocks in the book
mdbook test book -L ./target/debug/deps
```

**How it works:**
1. `cargo build` creates `libros_z-*.rlib` in `target/debug/deps/`
2. `mdbook test` extracts code blocks from markdown
3. `-L ./target/debug/deps` tells rustc where to find the library
4. Each code block compiles and runs (unless marked `no_run`)

**Verified:** ✅ Book tests pass.

### Step 4: Include Paths Work

Book chapters use:
```markdown
\`\`\`rust,no_run
{{#include ../../../ros-z/examples/demo_nodes/talker.rs}}
\`\`\`
```

**Path resolution:**
```
book/src/chapters/demo_talker.md
                  ↓ ../
book/src/
                  ↓ ../
book/
                  ↓ ../
ros-z/ (workspace root)
                  ↓ ros-z/examples/
ros-z/examples/demo_nodes/talker.rs  ✅
```

**Verified:** ✅ Include paths resolve correctly.

## Complete Test Commands

```bash
# Quick test (recommended)
./test-docs.sh

# Manual verification
cargo build
cargo test
cargo run --example demo_nodes_talker -- --help
mdbook test book -L ./target/debug/deps
mdbook serve book  # Preview at http://localhost:3000
```

## Why This Structure Works

### Standard Rust Pattern

This is the **recommended** way to organize Rust projects:

```toml
# In the library crate (ros-z/Cargo.toml)
[dependencies]
# All dependencies examples need

[[example]]
name = "demo_nodes_talker"
path = "examples/demo_nodes/talker.rs"
```

Benefits:
- ✅ Examples automatically have access to all dependencies
- ✅ `cargo build` builds everything
- ✅ `cargo run --example` works out of the box
- ✅ Standard structure everyone understands
- ✅ No special configuration needed

### Alternative (NOT Recommended)

Some suggest making examples a separate workspace member:

```
examples/
├── Cargo.toml  # Separate package
└── *.rs
```

Problems:
- ❌ Non-standard structure
- ❌ Breaks `{{#include}}` paths (need to update 20+ files)
- ❌ More complex `cargo` commands
- ❌ Duplicate dependency declarations
- ❌ Harder to maintain

**Verdict:** Stick with the current structure.

## CI Verification

GitHub Actions (`.github/workflows/docs.yml`) runs:

```yaml
- run: cargo build              # Builds library + examples
- run: cargo test               # Tests library
- run: mdbook test book -L ./target/debug/deps  # Tests book
```

This ensures:
1. Library compiles
2. Tests pass
3. **All documentation examples compile and work**
4. Book can be built

## Troubleshooting

### "can't find crate for `ros_z`" in mdbook test

**Solution:** Run `cargo build` first.

```bash
cargo build
mdbook test book -L ./target/debug/deps
```

### "cannot find value `X` in this scope" in example

**Solution:** Add the dependency to `ros-z/Cargo.toml`:

```toml
[dependencies]
X = "version"
```

Then rebuild:
```bash
cargo build
mdbook test book -L ./target/debug/deps
```

### Example uses ros-z-msgs but won't compile

**Solution:** ros-z-msgs is a dev-dependency. Build from workspace root:

```bash
# From workspace root (not ros-z/)
cargo build --example demo_nodes_talker
```

Or add the feature:
```bash
cargo build --example z_srvcli --features external_msgs
```

## Summary

The current structure is **optimal** and follows Rust best practices:

| Requirement | Status | Implementation |
|-------------|--------|----------------|
| Examples can run | ✅ | `cargo run --example name` |
| Examples have deps | ✅ | In `ros-z/Cargo.toml` |
| Book can test | ✅ | `mdbook test -L ./target/debug/deps` |
| Include paths work | ✅ | `{{#include ../../../ros-z/examples/...}}` |
| CI validates | ✅ | `.github/workflows/docs.yml` |
| Standard structure | ✅ | Standard Rust project layout |

**No changes needed!** The workflow is already correct. 🎉

## Quick Reference

```bash
# Run an example
cargo run --example demo_nodes_talker

# Test documentation
./test-docs.sh

# Full workflow
cargo build && cargo test && mdbook test book -L ./target/debug/deps
```
