# Contributing to ros-z

Thank you for your interest in contributing to ros-z! This document provides guidelines and workflows for contributors.

## Getting Started

### Prerequisites

1. **Rust toolchain**: Install via [rustup](https://rustup.rs/)
2. **Nix** (optional but recommended): For reproducible development environment
3. **mdbook**: For documentation (included in nix environment)

### Setting Up Your Development Environment

#### Using Nix (Recommended)

```bash
# Clone the repository
git clone https://github.com/ZettaScaleLabs/ros-z.git
cd ros-z

# Enter the development shell (includes all dependencies)
nix develop
# or if direnv is installed
direnv allow
```

#### Without Nix

```bash
# Install Rust
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh

# Install mdbook
cargo install mdbook

# Install other dependencies as needed
```

## Development Workflow

### 1. Build the Project

```bash
# Build the entire workspace
cargo build

# Build specific packages
cargo build -p ros-z
cargo build -p ros-z-msgs
```

### 2. Run Tests

```bash
# Run all tests
cargo test

# Run tests for specific package
cargo test -p ros-z

# Run integration tests
cargo test -p ros-z-tests --features ros-msgs
```

### 3. Test Documentation

When making changes to examples or documentation:

```bash
# Build the library first (required for mdbook test)
cargo build

# Test all code examples in the book
mdbook test book -L ./target/debug/deps

# Preview the book locally
mdbook serve book
```

Open <http://localhost:3000> to see the rendered documentation.

## Pre-Commit Checklist

Before submitting changes, ensure all of these pass:

```bash
# 1. Build succeeds
cargo build

# 2. All tests pass
cargo test

# 3. Documentation examples work (if examples changed)
mdbook test book -L ./target/debug/deps

# 4. Code is formatted
cargo fmt

# 5. No clippy warnings
cargo clippy --all-targets --all-features
```

If you're using the Nix environment, pre-commit hooks will automatically run these checks.

## Making Changes

### Adding New Features

When adding a new ros-z feature:

1. **Implement** the feature in `ros-z/src/`
2. **Add an example** to `ros-z/examples/` demonstrating the feature
3. **Update documentation**:
   - Add or update book chapter in `book/src/chapters/`
   - Use `{{#include ../../ros-z/examples/your_example.rs}}` to reference the example
   - Update `book/src/SUMMARY.md` if adding a new chapter
4. **Test everything**:

   ```bash
   cargo build
   cargo test
   mdbook test book -L ./target/debug/deps
   ```

### Updating Existing Features

When changing existing APIs:

1. **Update** the implementation in `ros-z/src/`
2. **Update** affected examples in `ros-z/examples/`
3. **Update** book chapters with new explanations (if needed)
4. **Run tests** to ensure everything still works
5. Make all changes in a **single commit**: "feat: new API + examples + docs"

### Documentation Guidelines

Documentation in ros-z follows a specific pattern:

#### Examples are the Single Source of Truth

- All code examples live in `ros-z/examples/` as complete, runnable programs
- Book chapters reference these examples using `{{#include}}` directives
- **Never** write inline code blocks >5 lines in markdown files
- Examples must compile and pass `mdbook test`

#### Example Template

Every example should follow this pattern:

```rust
use ros_z::prelude::*;

fn main() -> Result<()> {
    // Your example code here
    let ctx = ZContextBuilder::default().build()?;
    let node = ctx.create_node("example").build()?;
    // ... rest of example
    Ok(())
}
```

For examples that should also work as library functions (testable), use:

```rust
pub fn run_example(ctx: ZContext, /* other params */) -> Result<()> {
    // Implementation here
}

#[cfg(not(any(test, doctest)))]
fn main() -> Result<()> {
    // CLI argument parsing
    let ctx = ZContextBuilder::default().build()?;
    run_example(ctx /* pass params */)
}
```

#### Book Chapter Template

```markdown
# Feature Name

Brief introduction to the feature.

## Complete Example

\`\`\`rust,no_run
{{#include ../../../ros-z/examples/your_example.rs}}
\`\`\`

## Key Points

- **Line X**: Explanation of important line
- **Line Y**: Another important detail

## Usage

\`\`\`bash
cargo run --example your_example
\`\`\`
```

### Code Style

- Follow Rust standard formatting: `cargo fmt`
- Run clippy and fix warnings: `cargo clippy`
- Write clear, self-documenting code
- Add comments for complex logic
- Use descriptive variable names

### Commit Messages

Follow conventional commit format:

- `feat: add new feature`
- `fix: resolve bug in X`
- `docs: update documentation for Y`
- `refactor: restructure Z`
- `test: add tests for W`
- `chore: update dependencies`

## Submitting Changes

1. **Fork** the repository
2. **Create a branch** from `main` with a descriptive name
3. **Make your changes** following the guidelines above
4. **Test thoroughly** using the pre-commit checklist
5. **Commit** with clear, conventional commit messages
6. **Push** to your fork
7. **Open a Pull Request** with:
   - Clear description of changes
   - Reference to related issues (if any)
   - Screenshots/examples (if applicable)

## Pull Request Process

1. Ensure all CI checks pass
2. Request review from maintainers
3. Address review feedback
4. Once approved, maintainers will merge

## Project Structure

```text
ros-z/
├── ros-z/                  # Core library
│   ├── src/               # Production code
│   └── examples/          # Runnable examples (used by book)
├── ros-z-msgs/            # Generated message types
├── ros-z-codegen/         # Code generation utilities
├── rcl-z/                 # RCL C bindings
├── ros-z-tests/           # Integration tests
├── book/                  # mdBook documentation
│   ├── book.toml         # mdBook configuration
│   └── src/              # Book chapters
└── .github/workflows/     # CI/CD
```

## Maintenance Rules

### If-Then Flows

**IF adding new ros-z feature:**

- ADD `examples/new_feature.rs`
- ADD/UPDATE book chapter with `{{#include}}`
- RUN `cargo build && mdbook test book -L ./target/debug/deps`

**IF making API breaking change:**

- UPDATE `examples/*.rs` files
- UPDATE book text references
- Single commit: "feat: new API + examples + docs"

**IF mdbook test fails:**

- RUN `cargo clean && cargo build`
- CHECK `target/debug/deps` contains `libros_z-*.rlib`
- RE-RUN `mdbook test book -L ./target/debug/deps`

## Getting Help

- **Questions**: Open a [GitHub Discussion](https://github.com/ZettaScaleLabs/ros-z/discussions)
- **Bugs**: Open a [GitHub Issue](https://github.com/ZettaScaleLabs/ros-z/issues)
- **Chat**: Join the [Zenoh Discord](https://discord.gg/vSDSpqnbkm)

## License

By contributing to ros-z, you agree that your contributions will be licensed under the same license as the project (see LICENSE file).

## Code of Conduct

Be respectful, inclusive, and professional in all interactions. We're all here to make ROS 2 better!

Thank you for contributing to ros-z! 🦀
