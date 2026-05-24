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
# Conda boost workaround for secure-exec binaries (file capabilities)
# ----------------------------------------------------------------------------
#
# When step1_joint_pd has file capabilities (e.g. cap_sys_nice), glibc treats
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
        message(STATUS "panda_control: using conda boost filesystem: ${CONDA_BOOST_FILESYSTEM}")
    else()
        message(WARNING "panda_control: conda boost filesystem not found under $ENV{CONDA_PREFIX}/lib")
    endif()
endif()

# ----------------------------------------------------------------------------
# Step 1: Joint PD hold (standalone)
# ----------------------------------------------------------------------------
add_executable(step1_joint_pd src/step1_joint_pd.cpp)
target_link_libraries(step1_joint_pd
    PRIVATE
        Franka::Franka
        Eigen3::Eigen
        Threads::Threads)
if(CONDA_BOOST_FILESYSTEM)
    target_link_options(step1_joint_pd PRIVATE -Wl,--no-as-needed)
    target_link_libraries(step1_joint_pd PRIVATE ${CONDA_BOOST_FILESYSTEM})
endif()
target_compile_options(step1_joint_pd PRIVATE -O2)

# ----------------------------------------------------------------------------
# Step 3: Joint PD + explicit Coriolis compensation (standalone)
# ----------------------------------------------------------------------------
add_executable(step3_joint_pd_coriolis src/step3_joint_pd_coriolis.cpp)
target_link_libraries(step3_joint_pd_coriolis
    PRIVATE
        Franka::Franka
        Eigen3::Eigen
        Threads::Threads)
if(CONDA_BOOST_FILESYSTEM)
    target_link_options(step3_joint_pd_coriolis PRIVATE -Wl,--no-as-needed)
    target_link_libraries(step3_joint_pd_coriolis PRIVATE ${CONDA_BOOST_FILESYSTEM})
endif()
target_compile_options(step3_joint_pd_coriolis PRIVATE -O2)

# ----------------------------------------------------------------------------
# Step 4: Joint trajectory tracking (same law as step 3, time-varying q_des)
# ----------------------------------------------------------------------------
add_executable(step4_joint_traj src/step4_joint_traj.cpp)
target_link_libraries(step4_joint_traj
    PRIVATE
        Franka::Franka
        Eigen3::Eigen
        Threads::Threads)
if(CONDA_BOOST_FILESYSTEM)
    target_link_options(step4_joint_traj PRIVATE -Wl,--no-as-needed)
    target_link_libraries(step4_joint_traj PRIVATE ${CONDA_BOOST_FILESYSTEM})
endif()
target_compile_options(step4_joint_traj PRIVATE -O2)

# ----------------------------------------------------------------------------
# Step 5: Jacobian-transpose Cartesian PD (position-only, no Lambda)
# ----------------------------------------------------------------------------
add_executable(step5_cart_pd src/step5_cart_pd.cpp)
target_link_libraries(step5_cart_pd
    PRIVATE
        Franka::Franka
        Eigen3::Eigen
        Threads::Threads)
if(CONDA_BOOST_FILESYSTEM)
    target_link_options(step5_cart_pd PRIVATE -Wl,--no-as-needed)
    target_link_libraries(step5_cart_pd PRIVATE ${CONDA_BOOST_FILESYSTEM})
endif()
target_compile_options(step5_cart_pd PRIVATE -O2)

# ----------------------------------------------------------------------------
# Step 5b: Jacobian-transpose Cartesian PD (6D pose, no Lambda)
# ----------------------------------------------------------------------------
add_executable(step5b_cart_pose src/step5b_cart_pose.cpp)
target_link_libraries(step5b_cart_pose
    PRIVATE
        Franka::Franka
        Eigen3::Eigen
        Threads::Threads)
if(CONDA_BOOST_FILESYSTEM)
    target_link_options(step5b_cart_pose PRIVATE -Wl,--no-as-needed)
    target_link_libraries(step5b_cart_pose PRIVATE ${CONDA_BOOST_FILESYSTEM})
endif()
target_compile_options(step5b_cart_pose PRIVATE -O2)

# ----------------------------------------------------------------------------
# Utility: Read current joint positions (single readOnce call)
# ----------------------------------------------------------------------------
add_executable(read_current_q src/read_current_q.cpp)
target_link_libraries(read_current_q
    PRIVATE
        Franka::Franka
        Threads::Threads)
if(CONDA_BOOST_FILESYSTEM)
    target_link_options(read_current_q PRIVATE -Wl,--no-as-needed)
    target_link_libraries(read_current_q PRIVATE ${CONDA_BOOST_FILESYSTEM})
endif()
target_compile_options(read_current_q PRIVATE -O2)

# ----------------------------------------------------------------------------
# Future steps will add more executables here:
#   step2_task_pd, step3_osc, ...
# ----------------------------------------------------------------------------
