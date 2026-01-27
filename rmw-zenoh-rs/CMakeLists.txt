cmake_minimum_required(VERSION 3.8)
project(rmw_zenoh_rs)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(ament_index_cpp REQUIRED)
find_package(fastcdr CONFIG REQUIRED)
find_package(rcpputils REQUIRED)
find_package(rcutils REQUIRED)
find_package(rosidl_typesupport_fastrtps_c REQUIRED)
find_package(rosidl_typesupport_fastrtps_cpp REQUIRED)
find_package(rmw REQUIRED)

# Determine Cargo profile based on CMAKE_BUILD_TYPE
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  set(CARGO_BUILD_TYPE "debug")
  set(CARGO_BUILD_FLAG "")
else()
  set(CARGO_BUILD_TYPE "release")
  set(CARGO_BUILD_FLAG "--release")
endif()

set(GENERATED_LIB_FILE ${CMAKE_CURRENT_BINARY_DIR}/${CARGO_BUILD_TYPE}/librmw_zenoh_rs.so)

# Resolve symlinks to get the real workspace root (important when built via colcon symlink)
get_filename_component(REAL_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR} REALPATH)
get_filename_component(CARGO_WORKSPACE_DIR ${REAL_SOURCE_DIR}/.. ABSOLUTE)
message(STATUS "Cargo workspace directory: ${CARGO_WORKSPACE_DIR}")

# Build with Cargo - it discovers include/library paths from AMENT_PREFIX_PATH
add_custom_target(cargo_build_rmw_zenoh_rs ALL
  COMMAND ${CMAKE_COMMAND} -E env
          "AMENT_PREFIX_PATH=$ENV{AMENT_PREFIX_PATH}"
          cargo build ${CARGO_BUILD_FLAG} --manifest-path ${CMAKE_CURRENT_SOURCE_DIR}/Cargo.toml
          --target-dir ${CMAKE_CURRENT_BINARY_DIR}
  BYPRODUCTS ${GENERATED_LIB_FILE}
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/..
  COMMENT "Building Rust library with ${CARGO_BUILD_TYPE} profile (Cargo handles incremental builds)"
  VERBATIM
)

# Create a library target that wraps the Cargo-built library
add_library(rmw_zenoh_rs SHARED IMPORTED GLOBAL)
set_property(TARGET rmw_zenoh_rs PROPERTY IMPORTED_LOCATION ${GENERATED_LIB_FILE})
add_dependencies(rmw_zenoh_rs cargo_build_rmw_zenoh_rs)

configure_rmw_library(rmw_zenoh_rs)

register_rmw_implementation(
  "c:rosidl_typesupport_fastrtps_c:rosidl_typesupport_introspection_c"
  "cpp:rosidl_typesupport_fastrtps_cpp:rosidl_typesupport_introspection_cpp")

install(
  FILES ${GENERATED_LIB_FILE}
  DESTINATION lib
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
