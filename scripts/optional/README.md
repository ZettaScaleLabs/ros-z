# Optional Validation Scripts

These scripts are **not run in CI** and are intended for manual use when needed.

---

## Scripts

### `validate-cli-docs.nu`

**Purpose:** Validates that all `cargo run --example` commands in documentation actually work

**When to use:**

- After updating documentation
- Before releasing
- When modifying examples
- When adding new examples to docs

**Runtime:** ~10-15 minutes (compiles all 52 examples)

**Usage:**

```bash
./scripts/optional/validate-cli-docs.nu
```

**What it checks:**

- All examples mentioned in docs exist
- Examples accept documented arguments
- `--help` output matches documented flags

---

### `check-example-coverage.nu`

**Purpose:** Checks which examples are documented and which are missing from docs

**When to use:**

- After adding new examples
- During documentation audits
- Before releases

**Runtime:** ~1 minute

**Usage:**

```bash
./scripts/optional/check-example-coverage.nu
```

---

### `test-summary.nu`

**Purpose:** Quick view of test results from `_tmp/test-logs/`

**When to use:**

- After running `run-all-tests.nu`
- To quickly check which tests failed
- To view test output summaries

**Runtime:** Instant

**Usage:**

```bash
# View all test results
./scripts/optional/test-summary.nu

# Only show failed tests
./scripts/optional/test-summary.nu --failed-only

# Show more context (default 50 lines)
./scripts/optional/test-summary.nu --tail 100
```
