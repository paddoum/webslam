cmake_minimum_required(VERSION 3.16)
project(webslam CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT EMSCRIPTEN)
  message(FATAL_ERROR "Configure with emcmake: `emcmake cmake ..` (Emscripten toolchain required)")
endif()

# Eigen is header-only. M1 does not use it yet, but later milestones (M3/M4) do.
# Path is resolved by build.sh and passed in as EIGEN_INCLUDE_DIR.
if(DEFINED EIGEN_INCLUDE_DIR)
  include_directories(${EIGEN_INCLUDE_DIR})
endif()

# Place output where the web harness expects it (must be set before the target).
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/web/wasm)

add_executable(slam
  src/slam_engine.cpp
  src/orb.cpp
  src/two_view.cpp
  src/pnp.cpp
  src/ba.cpp
  src/map.cpp
  src/scale.cpp
  src/imu_preint.cpp
  src/vi_init.cpp
  src/vi_optimizer.cpp
  src/vi_window.cpp
  src/bindings.cpp
)

target_include_directories(slam PRIVATE ${CMAKE_SOURCE_DIR}/src)

# Emscripten link flags: build an ES6 module exposing an Embind interface.
set_target_properties(slam PROPERTIES
  OUTPUT_NAME "slam"
  LINK_FLAGS "\
    -lembind \
    -sMODULARIZE=1 \
    -sEXPORT_ES6=1 \
    -sEXPORT_NAME=createSlamModule \
    -sALLOW_MEMORY_GROWTH=1 \
    -sENVIRONMENT=web,worker,node \
    -sINITIAL_MEMORY=33554432 \
    -O3 \
  ")
