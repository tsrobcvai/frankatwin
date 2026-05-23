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
