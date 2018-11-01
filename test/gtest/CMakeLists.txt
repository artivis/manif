cmake_minimum_required(VERSION 2.8.8)
project(gtest_builder C CXX)

# We need thread support
#find_package(Threads REQUIRED)

# Enable ExternalProject CMake module
include(ExternalProject)

set(GTEST_FORCE_SHARED_CRT ON)
set(GTEST_DISABLE_PTHREADS OFF)

# For some reason I need to disable PTHREADS
# with g++ (Ubuntu 4.9.3-8ubuntu2~14.04) 4.9.3
# This is a known issue for MinGW :
# https://github.com/google/shaderc/pull/174
#if(MINGW)
  set(GTEST_DISABLE_PTHREADS ON)
#endif()

# Download GoogleTest
ExternalProject_Add(googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG        v1.8.x
  # TIMEOUT 1 # We'll try this
  CMAKE_ARGS -DCMAKE_ARCHIVE_OUTPUT_DIRECTORY_DEBUG:PATH=DebugLibs
  -DCMAKE_ARCHIVE_OUTPUT_DIRECTORY_RELEASE:PATH=ReleaseLibs
  -DCMAKE_CXX_FLAGS=${MSVC_COMPILER_DEFS}
  -Dgtest_force_shared_crt=${GTEST_FORCE_SHARED_CRT}
  -Dgtest_disable_pthreads=${GTEST_DISABLE_PTHREADS}
  -DBUILD_GTEST=ON
  PREFIX "${CMAKE_CURRENT_BINARY_DIR}"
  # Disable install step
  INSTALL_COMMAND ""
  UPDATE_DISCONNECTED 1 # 1: do not update googletest; 0: update googletest via github
)

# Get GTest source and binary directories from CMake project

# Specify include dir
ExternalProject_Get_Property(googletest source_dir)
set(GTEST_INCLUDE_DIRS ${source_dir}/googletest/include PARENT_SCOPE)

# Specify MainTest's link libraries
ExternalProject_Get_Property(googletest binary_dir)
set(GTEST_LIBS_DIR ${binary_dir}/googlemock/gtest PARENT_SCOPE)

# Create a libgtest target to be used as a dependency by test programs
add_library(libgtest IMPORTED STATIC GLOBAL)
add_dependencies(libgtest googletest)

# Set libgtest properties
set_target_properties(libgtest PROPERTIES
    "IMPORTED_LOCATION" "${binary_dir}/googlemock/gtest/libgtest.a"
    "IMPORTED_LINK_INTERFACE_LIBRARIES" "${CMAKE_THREAD_LIBS_INIT}"
)
