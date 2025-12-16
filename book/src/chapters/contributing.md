# Contributing

Thank you for your interest in contributing to ros-z!

## Development Workflow

### 1. Build the Project

```bash
cargo build
```

### 2. Run Tests

```bash
cargo test
```

### 3. Test Documentation

When making changes to examples or documentation:

```bash
# Test all code examples in the book
mdbook test book -L ./target/debug/deps

# Preview the book locally
mdbook serve book
```

## Pre-Commit Checklist

Before submitting changes, ensure:

- [ ] `cargo build` succeeds
- [ ] `cargo test` passes
- [ ] `mdbook test book -L ./target/debug/deps` passes (if examples changed)
- [ ] Examples use `\{{#include}}` (no inline code >5 lines)
- [ ] API changes are reflected in both examples and book text
- [ ] `mdbook serve` - visually verify rendering

## Making Changes

### Adding New Features

If adding a new ros-z feature:

1. Implement the feature in `ros-z/src/`
2. Add an example to `ros-z/examples/`
3. Add/update book chapter with `\{{#include}}` directive
4. Run the full test suite

### Updating Documentation

- Examples are the single source of truth
- Always use `\{{#include ../../examples/...}}` in book chapters
- Keep narrative text in book chapters focused and concise

### Code Style

- Follow Rust standard formatting (`cargo fmt`)
- Run clippy (`cargo clippy`)
- Write clear, self-documenting code
- Add comments for complex logic

## Submitting Changes

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Test thoroughly
5. Submit a pull request

## Questions?

- Open an issue on [GitHub](https://github.com/ZettaScaleLabs/ros-z/issues)
- Check existing issues and discussions

Thank you for contributing to ros-z!
