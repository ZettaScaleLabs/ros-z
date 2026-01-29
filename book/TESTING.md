# Testing the mdBook Examples

This document explains how to test that all code examples in the book compile and run correctly.

## The `mdbook test` Command

mdBook has a built-in test command that:

1. Extracts all Rust code blocks from markdown files
2. Compiles them as mini test programs
3. Links them against your library
4. Runs them to verify they work

## Running the Tests

### Prerequisites

You must build the library first so mdbook can link against it:

```bash
cargo build
```

This creates `libros_z-*.rlib` in `target/debug/deps/`.

### Run All Book Tests

```bash
mdbook test book -L ./target/debug/deps
```

**Explanation:**

- `mdbook test book` - Test all code blocks in the book
- `-L ./target/debug/deps` - Tell the Rust compiler where to find the compiled `ros-z` library

### Expected Output

On success, you'll see:

```text
2025-12-16 [INFO] (mdbook::book): Book building has started
2025-12-16 [INFO] (mdbook::book): Running the html backend
2025-12-16 [INFO] (mdbook::renderer::html_handlebars::search): Index built in ...
2025-12-16 [INFO] (mdbook::book): Running the test backend
running 15 tests
test book/src/chapters/demo_talker.md - demo_talker::rust_1 (line 11) ... ok
test book/src/chapters/demo_listener.md - demo_listener::rust_1 (line 11) ... ok
...
test result: ok. 15 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out
```

### On Failure

If an example fails to compile, you'll see detailed error messages:

```text
test book/src/chapters/example.md - example::rust_1 (line 5) ... FAILED

error[E0425]: cannot find value `xyz` in this scope
 --> /tmp/.tmpXYZ/main.rs:5:10
  |
5 |     let x = xyz;
     |             ^^^ not found in this scope
```

## Code Block Annotations

mdBook supports annotations on code blocks that control testing:

### Always Test (Default)

```markdown
\`\`\`rust use ros_z::prelude::\*; fn main() { } \`\`\`
```

### Don't Run (Compile Only)

Use `no_run` when the code needs to compile but shouldn't execute (e.g., runs forever):

```markdown
\`\`\`rust,no_run {{#include ../../../ros-z/examples/demo_nodes/talker.rs}} \`\`\`
```

Most of our examples use `no_run` because they're full programs with `main()` that run indefinitely.

### Don't Even Compile

Use `ignore` when showing pseudo-code or incomplete examples:

```markdown
\`\`\`rust,ignore // This won't be tested let qos = QosProfile { ... }; \`\`\`
```

### Combine Annotations

```markdown
\`\`\`rust,no_run,ignore // Won't run or compile \`\`\`
```

## Integration with CI

The GitHub Actions workflow (`.github/workflows/docs.yml`) automatically runs these tests:

```yaml
- run: cargo build
- run: cargo test
- run: mdbook test book -L ./target/debug/deps
```

This ensures all documentation examples stay in sync with the code.

## Common Issues and Solutions

### Issue: "can't find crate for `ros_z`"

**Solution:** Run `cargo build` first. The mdbook test command needs the compiled library.

```bash
cargo build
mdbook test book -L ./target/debug/deps
```

### Issue: "could not read file for link"

**Solution:** Check that the include path is correct and the file exists:

```bash
# From the book chapter:
{{#include ../../../ros-z/examples/your_file.rs}}

# The path should resolve to:
book/src/chapters/../../../ros-z/examples/your_file.rs
# Which is: ros-z/examples/your_file.rs
```

### Issue: Example compiles but fails at runtime

**Solution:** Most examples should use `,no_run` annotation since they're complete programs:

```markdown
\`\`\`rust,no_run {{#include ../../../ros-z/examples/demo_nodes/talker.rs}} \`\`\`
```

### Issue: "linking with `cc` failed"

**Solution:** The `-L ./target/debug/deps` flag is missing:

```bash
# Wrong
mdbook test book

# Correct
mdbook test book -L ./target/debug/deps
```

## Testing a Specific Chapter

To test just one chapter:

```bash
# Test only the quick_start chapter
mdbook test book -L ./target/debug/deps book/src/chapters/quick_start.md
```

## Manual Testing

You can also manually verify examples compile:

```bash
# Extract an example and test it
rustc --edition 2021 \
  -L target/debug/deps \
  --extern ros_z=target/debug/libros_z.rlib \
  book/extracted_example.rs
```

But `mdbook test` does this automatically for all examples.

## Pre-Commit Hook

Consider adding this to your pre-commit workflow:

```bash
#!/bin/bash
# .git/hooks/pre-commit

echo "Testing documentation examples..."
cargo build --lib || exit 1
mdbook test book -L ./target/debug/deps || exit 1
echo "All documentation examples passed!"
```

Or use the Nix pre-commit hooks (already configured in the development shell).

## Best Practices

1. **Always build first**: Run `cargo build` before `mdbook test`
2. **Use `no_run` for full programs**: Examples with `main()` that run indefinitely
3. **Use `ignore` sparingly**: Only for pseudo-code or intentionally incomplete examples
4. **Test locally before pushing**: Run the full workflow:

   ```bash
   cargo build
   cargo test
   mdbook test book -L ./target/debug/deps
   ```

5. **Keep examples simple**: Complex examples are harder to test in documentation

## Understanding Test Output

Each test corresponds to a code block:

```text
test book/src/chapters/demo_talker.md - demo_talker::rust_1 (line 11) ... ok
     ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^   ^^^^^^^^^^^^^^^^^^^^  ^^^^^^^^
     File path                           Test name             Line number
```

If a chapter has multiple code blocks, they'll be numbered: `rust_1`, `rust_2`, etc.

## Debugging Failed Tests

When a test fails:

1. **Check the file and line number** in the error message
2. **Extract the code block** from the markdown
3. **Try compiling manually**:

   ```bash
   # Create a test file
   cat > test.rs << 'EOF'
   use ros_z::prelude::*;
   fn main() {
       // Your code here
   }
   EOF

   # Compile it
   rustc --edition 2021 -L target/debug/deps --extern ros_z test.rs
   ```

4. **Fix the issue** in the example file (not the markdown!)
5. **Update the markdown** if needed (should auto-update if using `{{#include}}`)
6. **Re-run**: `mdbook test book -L ./target/debug/deps`

## Summary

The complete test workflow is:

```bash
# 1. Build the library
cargo build

# 2. Run unit tests
cargo test

# 3. Test documentation examples
mdbook test book -L ./target/debug/deps

# 4. (Optional) Preview the book
mdbook serve book
```

This ensures your documentation is always accurate and executable! 🎯
