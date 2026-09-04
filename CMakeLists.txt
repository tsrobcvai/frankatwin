cmake_minimum_required(VERSION 3.10)

project(frankatwin
    VERSION 0.2.0
    DESCRIPTION "FrankaTwin: 1 kHz Cartesian impedance controller for Franka FR3/Panda (libfranka) with a sim-to-real system-ID pipeline."
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
# When a conda-built libfranka is reused (e.g. the one shipped with deoxys), its
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
# Same workaround as deoxys's CMakeLists.txt.
if(DEFINED ENV{CONDA_PREFIX})
    set(CMAKE_EXE_LINKER_FLAGS
        "${CMAKE_EXE_LINKER_FLAGS} -L$ENV{CONDA_PREFIX}/lib -Wl,-rpath,$ENV{CONDA_PREFIX}/lib")
    set(CMAKE_SHARED_LINKER_FLAGS
        "${CMAKE_SHARED_LINKER_FLAGS} -L$ENV{CONDA_PREFIX}/lib -Wl,-rpath,$ENV{CONDA_PREFIX}/lib")
    message(STATUS "frankatwin: prepending CONDA_PREFIX/lib to linker search path: $ENV{CONDA_PREFIX}/lib")
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
# Pinocchio (required by libfranka >= 0.14)
# ----------------------------------------------------------------------------
#
# Starting with libfranka 0.14 (server version 8+, FR3 system >= 5.7.0), the
# robot dynamics (mass matrix, Coriolis, gravity, Jacobians) are computed with
# the Pinocchio library instead of the old internal model. Linking
# Franka::Franka then pulls in undefined pinocchio symbols (e.g.
# computeJointJacobians), so every executable that links libfranka must also
# link Pinocchio.
#
# Point find_package at the same Pinocchio used to build this libfranka, e.g.:
#   cmake -S . -B build \
#     -DCMAKE_PREFIX_PATH="/path/to/deoxys;/home/<user>/local/pinocchio-min"
set(PINOCCHIO_LIBS "")
if(Franka_VERSION VERSION_GREATER_EQUAL 0.14.0)
    find_package(pinocchio REQUIRED)
    set(PINOCCHIO_LIBS pinocchio::pinocchio pinocchio::pinocchio_parsers)
    message(STATUS "frankatwin: libfranka ${Franka_VERSION} (>= 0.14) -> linking Pinocchio for dynamics")
else()
    message(STATUS "frankatwin: libfranka ${Franka_VERSION} (< 0.14) -> Pinocchio not required")
endif()

# ----------------------------------------------------------------------------
# Conda boost workaround for secure-exec binaries (file capabilities)
# ----------------------------------------------------------------------------
#
# When osc_shm runs with file capabilities (e.g. cap_sys_nice), glibc treats
# it as secure-exec and ignores LD_LIBRARY_PATH. Some transitive dependencies
# (via pinocchio parsers) require boost_filesystem from conda.
#
# Linking boost_filesystem directly into our executables makes it a direct
# dependency resolved via this executable's RUNPATH (which already includes
# CONDA_PREFIX/lib), avoiding loader failures like:
#   libboost_filesystem.so.1.82.0: cannot open shared object file
set(CONDA_BOOST_FILESYSTEM "")
if(DEFINED ENV{CONDA_PREFIX})
    set(_CONDA_BOOST_CANDIDATE "$ENV{CONDA_PREFIX}/lib/libboost_filesystem.so.1.82.0")
    if(EXISTS "${_CONDA_BOOST_CANDIDATE}")
        set(CONDA_BOOST_FILESYSTEM "${_CONDA_BOOST_CANDIDATE}")
        message(STATUS "frankatwin: using conda boost filesystem: ${CONDA_BOOST_FILESYSTEM}")
    else()
        message(WARNING "frankatwin: conda boost filesystem not found under $ENV{CONDA_PREFIX}/lib")
    endif()
endif()

# ----------------------------------------------------------------------------
# Utility: Read current joint positions (single readOnce call)
# ----------------------------------------------------------------------------
add_executable(read_current_q src/read_current_q.cpp)
target_link_libraries(read_current_q
    PRIVATE
        Franka::Franka
        ${PINOCCHIO_LIBS}
        Threads::Threads)
if(CONDA_BOOST_FILESYSTEM)
    target_link_options(read_current_q PRIVATE -Wl,--no-as-needed)
    target_link_libraries(read_current_q PRIVATE ${CONDA_BOOST_FILESYSTEM})
endif()
target_compile_options(read_current_q PRIVATE -O2)

# ----------------------------------------------------------------------------
# Utility: Read current EE pose and emit a sidecar JSON
# ----------------------------------------------------------------------------
# Single readOnce() + model.pose(kEndEffector). Output is consumed by
# scripts/gen_excitation_traj.py / gen_chirp_traj.py via --base-sidecar, so
# excitation trajectories are anchored on the robot's *current* pose.
add_executable(read_current_pose src/read_current_pose.cpp)
target_link_libraries(read_current_pose
    PRIVATE
        Franka::Franka
        ${PINOCCHIO_LIBS}
        Eigen3::Eigen
        Threads::Threads)
if(CONDA_BOOST_FILESYSTEM)
    target_link_options(read_current_pose PRIVATE -Wl,--no-as-needed)
    target_link_libraries(read_current_pose PRIVATE ${CONDA_BOOST_FILESYSTEM})
endif()
target_compile_options(read_current_pose PRIVATE -O2)

# ----------------------------------------------------------------------------
# Utility: print configured EE/load dynamic params (m_ee / m_load / m_total)
# ----------------------------------------------------------------------------
# Single readOnce(); reports what Desk + setLoad already make the controller
# gravity-compensate, so you know what to put in robot.yaml `load.mass`.
add_executable(read_load src/read_load.cpp)
target_link_libraries(read_load
    PRIVATE
        Franka::Franka
        ${PINOCCHIO_LIBS}
        Threads::Threads)
if(CONDA_BOOST_FILESYSTEM)
    target_link_options(read_load PRIVATE -Wl,--no-as-needed)
    target_link_libraries(read_load PRIVATE ${CONDA_BOOST_FILESYSTEM})
endif()
target_compile_options(read_load PRIVATE -O2)

# ----------------------------------------------------------------------------
# POSIX shm + Python API
# ----------------------------------------------------------------------------
#
# `osc_shm` is the long-running 1 kHz controller. It implements a
# Jacobian-transpose 6D pose impedance law, reads setpoint/gains from a
# POSIX shm segment and publishes the robot state into the same segment.
#
# `move_to` is the one-shot reset utility. It links the vendored libfranka
# example MotionGenerator for joint-space min-jerk motion and uses libfranka's
# built-in CartesianPose motion type for EE-pose targets.
#
# Both link librt for shm_open / shm_unlink (POSIX shared memory).

# Vendored helper from libfranka examples (MotionGenerator + setDefaultBehavior).
add_library(frankatwin_examples_common STATIC src/examples_common.cpp)
target_include_directories(frankatwin_examples_common PUBLIC src)
target_link_libraries(frankatwin_examples_common
    PUBLIC
        Franka::Franka
        ${PINOCCHIO_LIBS}
        Eigen3::Eigen)
target_compile_options(frankatwin_examples_common PRIVATE -O2)

# ----------------------------------------------------------------------------
# osc_shm: long-running 1 kHz Cartesian impedance controller (shm-backed)
# ----------------------------------------------------------------------------
add_executable(osc_shm src/osc_shm.cpp)
target_include_directories(osc_shm PRIVATE src)
target_link_libraries(osc_shm
    PRIVATE
        Franka::Franka
        ${PINOCCHIO_LIBS}
        Eigen3::Eigen
        Threads::Threads
        rt)
if(CONDA_BOOST_FILESYSTEM)
    target_link_options(osc_shm PRIVATE -Wl,--no-as-needed)
    target_link_libraries(osc_shm PRIVATE ${CONDA_BOOST_FILESYSTEM})
endif()
target_compile_options(osc_shm PRIVATE -O2)

# ----------------------------------------------------------------------------
# move_to: one-shot reset via libfranka MotionGenerator (joint) or
# franka::CartesianPose (EE pose)
# ----------------------------------------------------------------------------
add_executable(move_to src/move_to.cpp)
target_include_directories(move_to PRIVATE src)
target_link_libraries(move_to
    PRIVATE
        frankatwin_examples_common
        Franka::Franka
        ${PINOCCHIO_LIBS}
        Eigen3::Eigen
        Threads::Threads)
if(CONDA_BOOST_FILESYSTEM)
    target_link_options(move_to PRIVATE -Wl,--no-as-needed)
    target_link_libraries(move_to PRIVATE ${CONDA_BOOST_FILESYSTEM})
endif()
target_compile_options(move_to PRIVATE -O2)
