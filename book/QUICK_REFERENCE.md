# mdBook Testing Quick Reference

## TL;DR - How to Test Examples

```bash
# Option 1: Use the test script (easiest)
./test-docs.sh

# Option 2: Manual testing
cargo build
mdbook test book -L ./target/debug/deps
```

## What Gets Tested?

The `mdbook test` command tests **every Rust code block** in your markdown files:

```markdown
# Your Chapter

\`\`\`rust,no_run {{#include ../../../ros-z/examples/your_example.rs}} \`\`\`
```

This code block will be:

1. Extracted by mdbook
2. Compiled with rustc
3. Linked against `libros_z.rlib` (from `target/debug/deps`)
4. Executed (unless marked with `,no_run`)

## Code Block Annotations

Control how code blocks are tested:

| Annotation | Effect | When to Use |
| --- | --- | --- |
| `rust` | Compile and run | Small, self-contained examples |
| `rust,no_run` | Compile only, don't run | Programs that run forever (like our examples) |
| `rust,ignore` | Don't compile or run | Pseudo-code, incomplete snippets |
| `bash` | Ignored by mdbook test | Shell commands |

## Example Annotations in Practice

### Full Program Example (Most Common)

```markdown
\`\`\`rust,no_run {{#include ../../../ros-z/examples/demo_nodes/talker.rs}} \`\`\`
```

Use `no_run` because talker runs indefinitely in a loop.

### Code Snippet That Should Compile

```markdown
\`\`\`rust,ignore use ros_z::qos::{QosProfile, QosHistory};

let qos = QosProfile { history: QosHistory::KeepLast(10), ..Default::default() }; \`\`\`
```

Use `ignore` when showing configuration examples that can't run standalone.

### Inline Documentation Code

```markdown
\`\`\`rust use ros_z::prelude::\*;

fn main() { let ctx = ZContextBuilder::default().build().unwrap(); assert!(ctx.is_valid()); } \`\`\`
```

No annotation needed - this will compile and run.

## The `-L` Flag is Critical

```bash
# ❌ WRONG - won't find ros-z library
mdbook test book

# ✅ CORRECT - links against compiled library
mdbook test book -L ./target/debug/deps
```

Without `-L ./target/debug/deps`, the test will fail with:

```text
error[E0463]: can't find crate for `ros_z`
```

## Testing Workflow

### Before Committing

```bash
cargo build
mdbook test book -L ./target/debug/deps
```

### CI/CD (GitHub Actions)

Already configured in `.github/workflows/docs.yml`:

```yaml
- run: cargo build
- run: cargo test
- run: mdbook test book -L ./target/debug/deps
```

Runs automatically on every push and pull request.

## Common Scenarios

### Adding a New Example

1. Create `ros-z/examples/my_feature.rs`
2. Add chapter `book/src/chapters/my_feature.md`
3. Include the example:

   ```markdown
   \`\`\`rust,no_run {{#include ../../../ros-z/examples/my_feature.rs}} \`\`\`
   ```

4. Test it:

   ```bash
   cargo build --example my_feature  # Verify example compiles
   mdbook test book -L ./target/debug/deps  # Verify book sees it
   ```

### Fixing a Failing Test

If `mdbook test` fails:

1. Check which file/line failed in the error
2. Fix the **example file** (not the markdown!)
3. Rebuild: `cargo build`
4. Re-test: `mdbook test book -L ./target/debug/deps`

### Testing One Chapter

```bash
mdbook test book -L ./target/debug/deps book/src/chapters/specific_chapter.md
```

## Integration with Development

### Pre-commit Hook

Add to `.git/hooks/pre-commit`:

```bash
#!/bin/bash
cargo build --lib || exit 1
mdbook test book -L ./target/debug/deps || exit 1
```

### Nix Development Shell

The nix environment includes pre-commit hooks that run these tests automatically.

### VSCode Integration

Add to `.vscode/tasks.json`:

```json
{
  "label": "Test mdBook",
  "type": "shell",
  "command": "./test-docs.sh",
  "group": "test"
}
```

## Benefits

✅ **Ensures examples compile** - Broken examples are caught immediately ✅ **Validates includes** - Missing files or wrong paths are detected ✅ **Maintains accuracy** - Documentation stays in sync with code ✅ **Automates verification** - No manual testing needed ✅ **CI integration** - Runs on every commit

## Learn More

- [TESTING.md](TESTING.md) - Comprehensive testing guide
- [README.md](README.md) - Development workflow
- [mdBook Documentation](https://rust-lang.github.io/mdBook/cli/test.html) - Official mdbook test docs

---

**Remember:** `mdbook test` is your safety net. Run it before every commit! 🛡️
