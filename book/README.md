# ros-z Documentation

This directory contains the mdBook documentation for ros-z.

## Prerequisites

mdbook is required to build and test the documentation. It's included in the development environment:

```bash
# Enter the nix development shell (if not already in it)
nix develop

# Or reload direnv
direnv allow
```

## Local Development Workflow

Follow these steps in order (as specified in the proposal):

### Quick Test (Recommended)

Use the provided test script:

```bash
./test-docs.sh
```

This runs the complete 4-step workflow. Add `--build` to also build the book HTML.

### Manual Workflow

#### 1. Build the Library

```bash
cargo build
```

This builds `libros_z-*.rlib` to `target/debug/deps`, which is required for mdbook to test the examples.

#### 2. Run Tests

```bash
cargo test
```

#### 3. Test Documentation Examples

```bash
mdbook test book -L ./target/debug/deps
```

This command:

- Tests all Rust code blocks in the book chapters
- Uses `-L ./target/debug/deps` to link against the compiled ros-z library
- Validates that all `{{#include}}` directives work correctly
- **This is the key test** that verifies examples can run

#### 4. Preview the Book (Optional)

```bash
mdbook serve book
```

Then open <http://localhost:3000> in your browser to see the rendered documentation.

## Understanding `mdbook test`

The `mdbook test` command is what validates that examples can run:

- Extracts all Rust code blocks from markdown files
- Compiles each one as a separate test program
- Links against your `ros-z` library using the `-L` flag
- Reports any compilation or runtime errors

See [TESTING.md](TESTING.md) for detailed information about testing.

## Project Structure

The current structure follows **standard Rust practices**:

```text
ros-z/                     # Workspace root
├── ros-z/                # Library crate
│   ├── Cargo.toml       # Has ALL dependencies (clap, tokio, etc.)
│   ├── src/             # Library code
│   └── examples/        # Examples (standard location)
│       ├── *.rs         # Top-level examples
│       └── demo_nodes/  # Subdirectory examples
└── book/                # Documentation
```

**Why this works:**

- ✅ Examples in `ros-z/examples/` have access to all dependencies
- ✅ `cargo run --example name` works out of the box
- ✅ `mdbook test` finds examples via `{{#include ../../../crates/ros-z/examples/...}}`
- ✅ Standard structure - no special configuration needed

See [WORKFLOW_VERIFICATION.md](WORKFLOW_VERIFICATION.md) for proof that this works.

## Building the Book

To build the static HTML:

```bash
mdbook build book
```

The output will be in `book/book/` directory.

## Book Structure

```text
book/
├── book.toml           # mdBook configuration
└── src/
    ├── SUMMARY.md      # Table of contents
    ├── introduction.md # Introduction page
    └── chapters/       # Chapter markdown files
```

## Writing Documentation

### Using Examples

Always reference examples using `{{#include}}`:

```markdown
{{#include ../../ros-z/examples/demo_nodes/talker.rs}}
```

### Adding New Chapters

1. Create the chapter markdown file in `book/src/chapters/`
2. Add it to `SUMMARY.md`
3. Use `{{#include}}` to reference example code
4. Run the test workflow to validate

## CI/CD

The documentation is automatically built and published on GitHub Pages when changes are pushed to the `main` branch. See `.github/workflows/docs.yml` for details.

## Troubleshooting

### "mdbook not found"

Make sure you're in the nix development shell:

```bash
direnv allow
# or
nix develop
```

### "can't find crate for `ros_z`"

Make sure you ran `cargo build` first. The mdbook test command needs the compiled library in `target/debug/deps`.

### Examples Don't Compile

1. Check that the example file exists at the path specified in `{{#include}}`
2. Verify the example compiles: `cargo build --example example_name`
3. Run `mdbook test book -L ./target/debug/deps` to see detailed errors
