cmake_minimum_required(VERSION 3.5.1)
cmake_policy(SET CMP0048 NEW)

project(manif VERSION 0.0.3 LANGUAGES CXX)

# Get the project name in capital letters
# for the lib namespace
string(TOUPPER ${PROJECT_NAME} PROJECT_NAME_CAPS)

# Check that the compoiler has c++11 support
# Crash otherwise.

if(NOT MSVC)
  include(CheckCXXCompilerFlag)
  CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
  if(COMPILER_SUPPORTS_CXX11)
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has C++11 support.")
  else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} "
      "has no C++11 support. Please use a different C++ compiler.")
  endif()
endif()

# Finding Eigen is somewhat complicated.
# First we look for the Eigen3 cmake module
# provided by the libeigen3-dev on newer Ubuntu. If that fails, then we
# fall-back to the version provided in the cmake/modules.
find_package(Eigen3 QUIET)

if(NOT EIGEN3_FOUND)
  list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake/modules)
  find_package(Eigen3 REQUIRED)
endif()

# Note that eigen 3.2 (on Ubuntu Wily) only provides EIGEN3_INCLUDE_DIR,
# not EIGEN3_INCLUDE_DIRS, so we have to set the latter from the former.
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

# Necessary for Ubuntu 16.04's god-awful FindEigen3.cmake.
if((NOT (DEFINED EIGEN3_VERSION)) AND (DEFINED EIGEN3_VERSION_STRING))
  set(EIGEN3_VERSION ${EIGEN3_VERSION_STRING})
endif()

if(${EIGEN3_VERSION} VERSION_EQUAL "3.3.6")
  message(WARNING "Eigen3 version ${EIGEN3_VERSION} found in ${EIGEN3_INCLUDE_DIRS},"
                  "but this version has a [bug](http://eigen.tuxfamily.org/bz/show_bug.cgi?id=1643)")
elseif(${EIGEN3_VERSION} VERSION_LESS "3.3.8")
  message(WARNING "Eigen3 version ${EIGEN3_VERSION} found in ${EIGEN3_INCLUDE_DIRS}. "
                  "Beware that the move semantic has a bug and resolves to a simple copy.")
endif()

# Options. Turn on with 'cmake -DBUILD_TESTING=ON'.
# catkin build manif --cmake-args -DBUILD_TESTING=ON -DBUILD_EXAMPLES=ON
option(BUILD_TESTING "Build all tests." OFF)
option(ENABLE_CPPCHECK "Enable cppcheck." OFF)
option(BUILD_EXAMPLES "Build all examples." OFF)
option(BUILD_BENCHMARKING "Build all benchmarks." OFF)
option(BUILD_PYTHON_BINDINGS "Build Python bindings with pybind11." OFF)
option(BUILD_TESTING_PYTHON "Build Python tests only." OFF)

if (BUILD_PYTHON_BINDINGS)
  find_package(pybind11 REQUIRED)
  add_subdirectory(python)
endif()

###########
## Build ##
###########

add_library(${PROJECT_NAME} INTERFACE)

# @todo export c++11 dependency
# @note A per-feature basis does
# not seems to work

# CMake 3.8 ...
#set_target_properties(${PROJECT_NAME} PROPERTIES
#  INTERFACE_COMPILE_FEATURES cxx_std_11
#)

option(USE_SYSTEM_WIDE_TL_OPTIONAL "Use a system wide installation of tl-optional" OFF)

find_package(tl-optional 1.0.0 QUIET)
if(USE_SYSTEM_WIDE_TL_OPTIONAL AND NOT tl-optional_FOUND)
  message(FATAL_ERROR "Could not find required package tl-optional")
endif()

target_include_directories(${PROJECT_NAME} INTERFACE
  "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include$<$<NOT:$<BOOL:${tl-optional_FOUND}>>:;${PROJECT_SOURCE_DIR}/external/tl>>"
  "$<INSTALL_INTERFACE:include>"
)
target_link_libraries(${PROJECT_NAME} INTERFACE "$<$<BOOL:${tl-optional_FOUND}>:tl::optional>")

# Add Eigen interface dependency, depending on available cmake info
if(TARGET Eigen3::Eigen)
  target_link_libraries(${PROJECT_NAME} INTERFACE Eigen3::Eigen)
  set(Eigen3_DEPENDENCY "find_dependency(Eigen3 ${Eigen3_VERSION})")
else(TARGET Eigen3::Eigen)
  target_include_directories(${PROJECT_NAME} SYSTEM INTERFACE ${EIGEN3_INCLUDE_DIRS})
endif(TARGET Eigen3::Eigen)

# Add tl-optional interface dependency if enabled
if(tl-optional_FOUND)
  set(tl-optional_DEPENDENCY "find_dependency(tl-optional)")
else()
  set(tl-optional_DEPENDENCY "")
endif()

if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
  target_compile_options(${PROJECT_NAME} INTERFACE -ftemplate-depth=512)
elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
  target_compile_options(${PROJECT_NAME} INTERFACE /bigobj)
endif()

#############
## Install ##
#############

include(CMakePackageConfigHelpers)
include(GNUInstallDirs)

set(config_install_dir "share/${PROJECT_NAME}/cmake")
set(generated_dir "${CMAKE_CURRENT_BINARY_DIR}/generated")

# Targets:
install(
  TARGETS  ${PROJECT_NAME}
  EXPORT   ${PROJECT_NAME}Targets
  INCLUDES DESTINATION include
)

install(
  EXPORT      ${PROJECT_NAME}Targets
  NAMESPACE   ${PROJECT_NAME_CAPS}::
  DESTINATION "${config_install_dir}"
)

export(
  TARGETS ${PROJECT_NAME}
  NAMESPACE ${PROJECT_NAME_CAPS}::
  FILE ${PROJECT_NAME}Targets.cmake
)

export(PACKAGE ${PROJECT_NAME})

## Configuration

#
configure_package_config_file(
  "${PROJECT_SOURCE_DIR}/cmake/${PROJECT_NAME}Config.cmake.in"
  "${generated_dir}/${PROJECT_NAME}Config.cmake"
  INSTALL_DESTINATION "${config_install_dir}"
)

# Configure '<PROJECT-NAME>ConfigVersion.cmake'
write_basic_package_version_file(
  "${generated_dir}/${PROJECT_NAME}ConfigVersion.cmake"
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY AnyNewerVersion
  ARCH_INDEPENDENT
)

# Config
install(
  FILES
    "${generated_dir}/${PROJECT_NAME}ConfigVersion.cmake"
    "${generated_dir}/${PROJECT_NAME}Config.cmake"
  DESTINATION "${config_install_dir}"
)

# Headers:

# if tl-optional is not found manif will install its own version
if(NOT tl-optional_FOUND)
  install(
    FILES "${PROJECT_SOURCE_DIR}/external/tl/tl/optional.hpp"
    DESTINATION include/tl
  )
endif()

install(
  DIRECTORY "${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}"
  DESTINATION include
)

# Test utils
install(
  FILES
    "${PROJECT_SOURCE_DIR}/test/gtest_eigen_utils.h"
    "${PROJECT_SOURCE_DIR}/test/gtest_manif_utils.h"
  DESTINATION "include/${PROJECT_NAME}/gtest"
)

##############
## Examples ##
##############

if(BUILD_EXAMPLES)

  add_subdirectory(examples)

endif(BUILD_EXAMPLES)

#############
## Testing ##
#############

# ------------------------------------------------------------------------------
# Coverage
# ------------------------------------------------------------------------------

if(ENABLE_COVERAGE)
  add_definitions(-DMANIF_COVERAGE_ENABLED)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g ")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fprofile-arcs")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ftest-coverage")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --coverage")
endif()

# ------------------------------------------------------------------------------
# CppCheck
# ------------------------------------------------------------------------------

if(ENABLE_CPPCHECK)

  # Find CppCheck executable
  find_program(CPPCHECK "cppcheck")
  if(CPPCHECK)

      # Set export commands on
      # set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

      list(APPEND CPPCHECK_ARGS
        --enable=all
        --std=c++11
        # --verbose
        --quiet
        # --check-config
        --xml-version=2
        --language=c++
        # Comment the line below to run cppcheck-html
        --error-exitcode=1
        --inline-suppr
        --suppress=*:*googletest-*
        --suppress=missingIncludeSystem
        --suppress=missingInclude
        --suppress=unmatchedSuppression:*
        --suppress=syntaxError:*tangent_base.h:370
        --suppress=unknownMacro:*test/common_tester.h
        --suppress=unknownMacro:*test/ceres/ceres_test_utils.h
        --suppress=constStatement:*examples*
        --suppress=constStatement:*gtest_misc.cpp:35
        --suppress=constStatement:*gtest_misc.cpp:43
        --suppress=unreadVariable:*gtest_se2_ceres_autodiff.cpp:101
        --suppress=unreadVariable:*bundle_sam.cpp:94
        --suppress=unreadVariable:*bundle_sam.cpp:124
        # Uncomment the line below to run cppcheck-html
        # --output-file=${CMAKE_BINARY_DIR}/cppcheck_results.xml
        ${CMAKE_SOURCE_DIR}/include
        ${CMAKE_SOURCE_DIR}/examples
        ${CMAKE_SOURCE_DIR}/test
      )

      add_custom_target(run-cppcheck
          COMMAND ${CPPCHECK} ${CPPCHECK_ARGS}
          COMMENT "Generate cppcheck report for the project"
      )

      find_program(CPPCHECK_HTML "cppcheck-htmlreport")
      if(CPPCHECK_HTML)
          add_custom_target(cppcheck-html
              COMMAND ${CPPCHECK_HTML}
              --title=${CMAKE_PROJECT_NAME}
              --file=${CMAKE_BINARY_DIR}/cppcheck_results.xml
              --report-dir=${CMAKE_BINARY_DIR}/cppcheck_results
              --source-dir=${CMAKE_SOURCE_DIR}
              COMMENT "Convert cppcheck report to HTML output"
          )
          ADD_DEPENDENCIES(cppcheck-html run-cppcheck)
      endif()
  endif()

endif()

# ------------------------------------------------------------------------------
# Valgrind
# ------------------------------------------------------------------------------

if(ENABLE_VALGRIND)
  # ctest -T memcheck
  include(CTest)
  set(MEMORYCHECK_COMMAND_OPTIONS "${MEMORYCHECK_COMMAND_OPTIONS} --leak-check=full")
  set(MEMORYCHECK_COMMAND_OPTIONS "${MEMORYCHECK_COMMAND_OPTIONS} --track-fds=yes")
  set(MEMORYCHECK_COMMAND_OPTIONS "${MEMORYCHECK_COMMAND_OPTIONS} --trace-children=yes")
  set(MEMORYCHECK_COMMAND_OPTIONS "${MEMORYCHECK_COMMAND_OPTIONS} --show-reachable=yes")
  set(MEMORYCHECK_COMMAND_OPTIONS "${MEMORYCHECK_COMMAND_OPTIONS} --error-exitcode=1")
endif()

# ------------------------------------------------------------------------------
# Unit Tests
# ------------------------------------------------------------------------------

if(BUILD_TESTING)

  enable_testing()
  add_subdirectory(test)

endif(BUILD_TESTING)

if(BUILD_PYTHON_BINDINGS AND (BUILD_TESTING OR BUILD_TESTING_PYTHON))

  enable_testing()
  add_subdirectory(test/python)

endif()

# ------------------------------------------------------------------------------
# Benchmark
# ------------------------------------------------------------------------------

if(BUILD_BENCHMARKING)

  add_subdirectory(benchmark)

endif(BUILD_BENCHMARKING)
