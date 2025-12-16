#!/bin/bash
# Test script for documentation examples
# This runs the complete 4-step workflow from the proposal

set -e  # Exit on any error

echo "📚 Testing ros-z documentation"
echo "================================"
echo ""

# Step 1: Build the library
echo "1️⃣  Building library..."
cargo build --lib
echo "✅ Build complete"
echo ""

# Step 2: Run unit tests
echo "2️⃣  Running unit tests..."
cargo test --lib
echo "✅ Unit tests passed"
echo ""

# Step 3: Test documentation examples
echo "3️⃣  Testing documentation examples..."
mdbook test book -L ./target/debug/deps
echo "✅ Documentation tests passed"
echo ""

# Step 4: Build the book (optional)
if [ "$1" = "--build" ]; then
    echo "4️⃣  Building book..."
    mdbook build book
    echo "✅ Book built successfully"
    echo "📖 Output: book/book/index.html"
    echo ""
fi

echo "================================"
echo "🎉 All documentation tests passed!"
echo ""
echo "To preview the book locally:"
echo "  mdbook serve book"
echo ""
