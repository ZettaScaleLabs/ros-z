cmake_minimum_required(VERSION 3.16)
project(cyclonedds_latency)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# set(CYCLONEDDS_DIR "/home/circle/Workings/ZettaScale/src/cyclonedds/.local/install")

# include_directories(${CYCLONEDDS_DIR}/include)
# link_directories(${CYCLONEDDS_DIR}/lib64)

if(NOT TARGET CycloneDDS::ddsc)
  find_package(CycloneDDS REQUIRED)
endif()

idlc_generate(TARGET Message_lib FILES Message.idl)

add_executable(sender src/sender.cpp)
add_executable(receiver src/receiver.cpp)

target_link_libraries(sender CycloneDDS::ddsc Message_lib)
target_link_libraries(receiver CycloneDDS::ddsc Message_lib)
