cmake_minimum_required(VERSION 3.10)

project(panda_control
    VERSION 0.1.0
    DESCRIPTION "From-scratch Franka controller stack (libfranka + small Python wrappers, hybrid)."
    LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

add_compile_options(-Wall -Wextra -Wpedantic)

# ----------------------------------------------------------------------------
# Conda libstdc++ link fix
# ----------------------------------------------------------------------------
#
# When deoxys's libfranka is reused (the usual setup on this NUC), its
# FrankaConfig.cmake declares `fmt::fmt` as a public link dependency.
# find_package(Franka) -> find_package(fmt) resolves to conda's libfmt,
# which was built with GCC 11+ and requires GLIBCXX_3.4.29. The system
# /usr/bin/c++ (Ubuntu 9.4) links against /usr/lib/.../libstdc++.so.6
# which only goes up to GLIBCXX_3.4.28, so ld fails on libfmt's symbols.
#
# Mirror deoxys's trick: push conda's lib to the front of the linker's
# search path, so the implicit -lstdc++ resolves to conda's libstdc++
# (GLIBCXX_3.4.34) instead of the system one.
#
# Reference: isaaclab_rollout/deoxys_control/deoxys/CMakeLists.txt:28-29
if(DEFINED ENV{CONDA_PREFIX})
    set(CMAKE_EXE_LINKER_FLAGS
        "${CMAKE_EXE_LINKER_FLAGS} -L$ENV{CONDA_PREFIX}/lib -Wl,-rpath,$ENV{CONDA_PREFIX}/lib")
    set(CMAKE_SHARED_LINKER_FLAGS
        "${CMAKE_SHARED_LINKER_FLAGS} -L$ENV{CONDA_PREFIX}/lib -Wl,-rpath,$ENV{CONDA_PREFIX}/lib")
    message(STATUS "panda_control: prepending CONDA_PREFIX/lib to linker search path: $ENV{CONDA_PREFIX}/lib")
endif()

# ----------------------------------------------------------------------------
# Dependencies
# ----------------------------------------------------------------------------
#
# libfranka can come from one of two places:
#   1. System install:    sudo apt install libfranka  (or build & `cmake --install`)
#   2. deoxys-vendored:   cmake -DCMAKE_PREFIX_PATH=/path/to/deoxys/build/libfranka ..
#
# Both produce a `Franka::Franka` imported target via find_package(Franka).
find_package(Franka REQUIRED)
find_package(Eigen3 REQUIRED)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

# ----------------------------------------------------------------------------
# Step 1: Joint PD hold (standalone)
# ----------------------------------------------------------------------------
add_executable(step1_joint_pd src/step1_joint_pd.cpp)
target_link_libraries(step1_joint_pd
    PRIVATE
        Franka::Franka
        Eigen3::Eigen
        Threads::Threads)
target_compile_options(step1_joint_pd PRIVATE -O2)

# ----------------------------------------------------------------------------
# Future steps will add more executables here:
#   step2_task_pd, step3_osc, ...
# ----------------------------------------------------------------------------
