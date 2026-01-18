#!/usr/bin/env bash
# Generate a minimal dependencies.json from ROS 2 packages

set -e

DISTRO=${1:-jazzy}
TEMP_DIR=$(mktemp -d)
ASSETS_DIR="ros-z-codegen/assets/$DISTRO"

echo "==> Generating dependencies.json for ROS $DISTRO..."

# Clone repositories
echo "==> Cloning repositories..."
git clone --depth 1 --branch $DISTRO https://github.com/ros2/common_interfaces.git "$TEMP_DIR/common_interfaces" 2>/dev/null
git clone --depth 1 --branch $DISTRO https://github.com/ros2/rcl_interfaces.git "$TEMP_DIR/rcl_interfaces" 2>/dev/null
git clone --depth 1 --branch $DISTRO https://github.com/ros2/example_interfaces.git "$TEMP_DIR/example_interfaces" 2>/dev/null
git clone --depth 1 --branch $DISTRO https://github.com/ros2/demos.git "$TEMP_DIR/demos" 2>/dev/null
git clone --depth 1 --branch $DISTRO https://github.com/ros2/unique_identifier_msgs.git "$TEMP_DIR/unique_identifier_msgs" 2>/dev/null

# Function to extract dependencies from package.xml
extract_deps() {
    local package_xml="$1"

    if [ ! -f "$package_xml" ]; then
        echo "[]"
        return
    fi

    # Extract <depend> and <build_depend> tags, filter for message packages
    local deps=$(grep -oP '(?<=<depend>)[^<]+(?=</depend>)|(?<=<build_depend>)[^<]+(?=</build_depend>)' "$package_xml" 2>/dev/null | \
        grep -E '(msgs|interfaces)$' | \
        sort -u | \
        awk '{print "\"" $0 "\""}' | \
        paste -sd ',' || echo "")

    if [ -z "$deps" ]; then
        echo "[]"
    else
        echo "[$deps]"
    fi
}

# Process packages and build JSON
PACKAGES=(
    "builtin_interfaces:$TEMP_DIR/rcl_interfaces/builtin_interfaces"
    "service_msgs:$TEMP_DIR/rcl_interfaces/service_msgs"
    "action_msgs:$TEMP_DIR/rcl_interfaces/action_msgs"
    "std_msgs:$TEMP_DIR/common_interfaces/std_msgs"
    "geometry_msgs:$TEMP_DIR/common_interfaces/geometry_msgs"
    "sensor_msgs:$TEMP_DIR/common_interfaces/sensor_msgs"
    "nav_msgs:$TEMP_DIR/common_interfaces/nav_msgs"
    "example_interfaces:$TEMP_DIR/example_interfaces"
    "action_tutorials_interfaces:$TEMP_DIR/demos/action_tutorials/action_tutorials_interfaces"
    "test_msgs:$TEMP_DIR/rcl_interfaces/test_msgs"
    "unique_identifier_msgs:$TEMP_DIR/unique_identifier_msgs/unique_identifier_msgs"
)

# Generate JSON using jq
JSON_CONTENT='{'
JSON_CONTENT+='"format_version": "1.0",'
JSON_CONTENT+='"ros_distro": "'$DISTRO'",'
JSON_CONTENT+='"generated": "'$(date -u +%Y-%m-%dT%H:%M:%SZ)'",'
JSON_CONTENT+='"packages": {'

FIRST=true
for entry in "${PACKAGES[@]}"; do
    pkg_name="${entry%%:*}"
    pkg_path="${entry##*:}"

    if [ ! -d "$pkg_path" ]; then
        continue
    fi

    package_xml="$pkg_path/package.xml"
    deps=$(extract_deps "$package_xml")

    if [ "$FIRST" = true ]; then
        FIRST=false
    else
        JSON_CONTENT+=','
    fi

    JSON_CONTENT+='"'$pkg_name'": {"dependencies": '$deps'}'
done

JSON_CONTENT+='}}'

# Write and pretty-print
echo "$JSON_CONTENT" | jq '.' > "$ASSETS_DIR/dependencies.json"

# Cleanup
rm -rf "$TEMP_DIR"

echo ""
echo "==> Generated: $ASSETS_DIR/dependencies.json"
echo "==> Size: $(du -h "$ASSETS_DIR/dependencies.json" | cut -f1)"
echo ""
cat "$ASSETS_DIR/dependencies.json"
