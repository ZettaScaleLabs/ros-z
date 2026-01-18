#!/usr/bin/env bash
# Update vendored ROS 2 message definitions

set -e

DISTRO=${1:-jazzy}
TEMP_DIR=$(mktemp -d)
ASSETS_DIR="ros-z-codegen/assets/$DISTRO"

echo "==> Updating vendored ROS $DISTRO assets..."

# Clone common_interfaces
echo "==> Cloning ros2/common_interfaces..."
git clone --depth 1 --branch $DISTRO \
  https://github.com/ros2/common_interfaces.git \
  "$TEMP_DIR/common_interfaces"

# Clone rcl_interfaces
echo "==> Cloning ros2/rcl_interfaces..."
git clone --depth 1 --branch $DISTRO \
  https://github.com/ros2/rcl_interfaces.git \
  "$TEMP_DIR/rcl_interfaces"

# Clone example_interfaces
echo "==> Cloning ros2/example_interfaces..."
git clone --depth 1 --branch $DISTRO \
  https://github.com/ros2/example_interfaces.git \
  "$TEMP_DIR/example_interfaces" 2>/dev/null || true

# Clone demos for action_tutorials_interfaces
echo "==> Cloning ros2/demos..."
git clone --depth 1 --branch $DISTRO \
  https://github.com/ros2/demos.git \
  "$TEMP_DIR/demos" 2>/dev/null || true

# Clone unique_identifier_msgs
echo "==> Cloning ros2/unique_identifier_msgs..."
git clone --depth 1 --branch $DISTRO \
  https://github.com/ros2/unique_identifier_msgs.git \
  "$TEMP_DIR/unique_identifier_msgs" 2>/dev/null || true

# Clone unique_identifier_msgs
echo "==> Cloning ros2/unique_identifier_msgs..."
git clone --depth 1 --branch $DISTRO \
  https://github.com/ros2/unique_identifier_msgs.git \
  "$TEMP_DIR/unique_identifier_msgs" 2>/dev/null || true

# Clear old assets (but keep any non-symlink directories first)
echo "==> Clearing old assets..."
if [ -d "$ASSETS_DIR" ]; then
  # Remove symlinks first
  find "$ASSETS_DIR" -maxdepth 1 -type l -delete 2>/dev/null || true
  # Remove everything else
  rm -rf "$ASSETS_DIR"
fi
mkdir -p "$ASSETS_DIR"

# Copy packages from rcl_interfaces
echo "==> Copying rcl_interfaces packages..."
for pkg in builtin_interfaces action_msgs service_msgs; do
  if [ -d "$TEMP_DIR/rcl_interfaces/$pkg" ]; then
    echo "  - $pkg"
    cp -r "$TEMP_DIR/rcl_interfaces/$pkg" "$ASSETS_DIR/"
  fi
done

# Copy unique_identifier_msgs
if [ -d "$TEMP_DIR/unique_identifier_msgs/unique_identifier_msgs" ]; then
  echo "==> Copying unique_identifier_msgs..."
  cp -r "$TEMP_DIR/unique_identifier_msgs/unique_identifier_msgs" "$ASSETS_DIR/"
fi

# Copy packages from common_interfaces
echo "==> Copying common_interfaces packages..."
for pkg in std_msgs geometry_msgs sensor_msgs nav_msgs; do
  if [ -d "$TEMP_DIR/common_interfaces/$pkg" ]; then
    echo "  - $pkg"
    cp -r "$TEMP_DIR/common_interfaces/$pkg" "$ASSETS_DIR/"
  fi
done

# Copy example_interfaces
if [ -d "$TEMP_DIR/example_interfaces" ]; then
  echo "==> Copying example_interfaces..."
  cp -r "$TEMP_DIR/example_interfaces" "$ASSETS_DIR/"
fi

# Copy action_tutorials_interfaces from demos
if [ -d "$TEMP_DIR/demos/action_tutorials/action_tutorials_interfaces" ]; then
  echo "==> Copying action_tutorials_interfaces..."
  cp -r "$TEMP_DIR/demos/action_tutorials/action_tutorials_interfaces" "$ASSETS_DIR/"
fi

# Copy test_msgs if available
if [ -d "$TEMP_DIR/rcl_interfaces/test_msgs" ]; then
  echo "==> Copying test_msgs..."
  cp -r "$TEMP_DIR/rcl_interfaces/test_msgs" "$ASSETS_DIR/"
fi

# Copy unique_identifier_msgs
if [ -d "$TEMP_DIR/unique_identifier_msgs/unique_identifier_msgs" ]; then
  echo "==> Copying unique_identifier_msgs..."
  cp -r "$TEMP_DIR/unique_identifier_msgs/unique_identifier_msgs" "$ASSETS_DIR/"
fi

# Record provenance
echo "==> Recording provenance..."
cat > "$ASSETS_DIR/README.md" <<EOF
# ROS 2 Message Definitions - $DISTRO

**Last Updated**: $(date -u +%Y-%m-%d)
**ROS Distro**: $DISTRO

## Sources

- **common_interfaces**
  - Repository: https://github.com/ros2/common_interfaces
  - Branch: $DISTRO
  - Commit: $(git -C "$TEMP_DIR/common_interfaces" rev-parse HEAD)

- **rcl_interfaces**
  - Repository: https://github.com/ros2/rcl_interfaces
  - Branch: $DISTRO
  - Commit: $(git -C "$TEMP_DIR/rcl_interfaces" rev-parse HEAD)

- **example_interfaces**
  - Repository: https://github.com/ros2/example_interfaces
  - Branch: $DISTRO
  - Commit: $(git -C "$TEMP_DIR/example_interfaces" rev-parse HEAD 2>/dev/null || echo "N/A")

- **demos** (for action_tutorials_interfaces)
  - Repository: https://github.com/ros2/demos
  - Branch: $DISTRO
  - Commit: $(git -C "$TEMP_DIR/demos" rev-parse HEAD 2>/dev/null || echo "N/A")

- **unique_identifier_msgs**
  - Repository: https://github.com/ros2/unique_identifier_msgs
  - Branch: $DISTRO
  - Commit: $(git -C "$TEMP_DIR/unique_identifier_msgs" rev-parse HEAD 2>/dev/null || echo "N/A")

## Packages Included

$(find "$ASSETS_DIR" -mindepth 1 -maxdepth 1 -type d -exec basename {} \; | sort | sed 's/^/- /')

## Total Size

$(du -sh "$ASSETS_DIR" | cut -f1)

## License

Apache License 2.0 (from ROS 2 project)
EOF

# Cleanup
echo "==> Cleaning up..."
rm -rf "$TEMP_DIR"

# Report
echo ""
echo "==> Summary:"
echo "  Distro: $DISTRO"
echo "  Location: $ASSETS_DIR"
echo "  Packages: $(find "$ASSETS_DIR" -mindepth 1 -maxdepth 1 -type d | wc -l)"
echo "  Total size: $(du -sh "$ASSETS_DIR" | cut -f1)"
echo ""
echo "==> Next steps:"
echo "  1. Review changes: git diff $ASSETS_DIR"
echo "  2. Test build: cd ros-z-msgs && cargo build"
echo "  3. Commit: git add $ASSETS_DIR && git commit -m 'Update vendored $DISTRO assets'"

# Clean up unnecessary files (keep only .msg, .srv, .action)
echo "==> Removing unnecessary files..."
find "$ASSETS_DIR" -type f ! \( -name "*.msg" -o -name "*.srv" -o -name "*.action" \) -delete
find "$ASSETS_DIR" -type d -empty -delete

# Generate dependencies.json
echo "==> Generating dependencies.json..."
bash "$(dirname "$0")/generate-dependencies-json.sh" "$DISTRO"
