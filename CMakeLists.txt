cmake_minimum_required(VERSION 2.8.12)
project(manif CXX)

# Get the project name in capital letters
# for the lib namespace
string(TOUPPER ${PROJECT_NAME} PROJECT_NAME_CAPS)

# Check that the compoiler has c++11 support
# Crash otherwise.
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
if(COMPILER_SUPPORTS_CXX11)
  message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has C++11 support.")
else()
  message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} "
    "has no C++11 support. Please use a different C++ compiler.")
endif()

# Set search directory for looking for our custom CMake scripts
# to look Eigen3.
LIST(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake/modules)

find_package(Eigen3 REQUIRED)

# Options. Turn on with 'cmake -DBUILD_TESTING=ON'.
# catkin build manif --cmake-args -DBUILD_TESTING=ON -BUILD_EXAMPLES=ON
option(BUILD_TESTING "Build all tests." OFF)
option(ENABLE_CPPCHECK "Enable cppcheck." OFF)
option(BUILD_EXAMPLES "Build all examples." OFF)

# Do a backup
set(CMAKE_REQUIRED_FLAGS_BK ${CMAKE_REQUIRED_FLAGS})
include(CheckCXXSourceCompiles)
set(CMAKE_REQUIRED_FLAGS "-std=c++11")
# Check if compiler support constexpr functions
CHECK_CXX_SOURCE_COMPILES(
"
  constexpr void f() {};
  int main(void) { return 0; }
"
_MANIF_COMPILER_SUPPORTS_CONSTEXPR_VOID_)
set(CMAKE_REQUIRED_FLAGS ${CMAKE_REQUIRED_FLAGS_BK})

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

target_include_directories(${PROJECT_NAME} INTERFACE
  "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include;${PROJECT_SOURCE_DIR}/external/lt>"
  "$<INSTALL_INTERFACE:include>"
)

# Add Eigen interface dependency, depending on available cmake info
if(TARGET Eigen3::Eigen)
  target_link_libraries(${PROJECT_NAME} INTERFACE Eigen3::Eigen)
  set(Eigen3_DEPENDENCY "find_dependency(Eigen3 ${Eigen3_VERSION})")
else(TARGET Eigen3::Eigen)
  target_include_directories(${PROJECT_NAME} SYSTEM INTERFACE ${EIGEN3_INCLUDE_DIR})
endif(TARGET Eigen3::Eigen)

if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
  target_compile_options(${PROJECT_NAME} INTERFACE -ftemplate-depth=512)
elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
  # GCC is not strict enough by default, so enable most of the warnings.
  target_compile_options(${PROJECT_NAME} INTERFACE
    -Werror=all
    -Werror=extra
    -Wno-unknown-pragmas
    -Wno-sign-compare
    -Wno-unused-parameter
    -Wno-missing-field-initializers
  )
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
  VERSION 0.0.0
  COMPATIBILITY AnyNewerVersion
)

# Config
install(
  FILES       "${generated_dir}/${PROJECT_NAME}ConfigVersion.cmake"
              "${generated_dir}/${PROJECT_NAME}Config.cmake"
  DESTINATION "${config_install_dir}"
)

# Headers:
install(
  FILES "${PROJECT_SOURCE_DIR}/external/lt/lt/optional.hpp"
  DESTINATION include/lt
)

install(
  DIRECTORY "${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}"
  DESTINATION include
)

install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}
#  DESTINATION "${config_install_dir}"
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
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g ")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fprofile-arcs")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ftest-coverage")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --coverage")
endif()

# ------------------------------------------------------------------------------
# CppCheck
# ------------------------------------------------------------------------------

# if(ENABLE_CPPCHECK)
#
#   list(APPEND CPPCHECK_ARGS
#     --enable=warning,style,performance,portability,unusedFunction
#     --std=c++11
#     --verbose
#     --error-exitcode=1
#     --language=c++
#     -DMAIN=main
#     -I ${CMAKE_SOURCE_DIR}/include
#       ${CMAKE_SOURCE_DIR}/include/*.h
#       #${CMAKE_SOURCE_DIR}/examples/*.cpp
#       ${CMAKE_SOURCE_DIR}/test/*.cpp
#   )
#
#
#   add_custom_target(check
#     COMMAND cppcheck ${CPPCHECK_ARGS}
#     COMMENT "running cppcheck"
#   )
#
# endif()

# ------------------------------------------------------------------------------
# Valgrind
# ------------------------------------------------------------------------------

# set(MEMORYCHECK_COMMAND_OPTIONS "${MEMORYCHECK_COMMAND_OPTIONS} --leak-check=full")
# set(MEMORYCHECK_COMMAND_OPTIONS "${MEMORYCHECK_COMMAND_OPTIONS} --track-fds=yes")
# set(MEMORYCHECK_COMMAND_OPTIONS "${MEMORYCHECK_COMMAND_OPTIONS} --trace-children=yes")
# set(MEMORYCHECK_COMMAND_OPTIONS "${MEMORYCHECK_COMMAND_OPTIONS} --error-exitcode=1")

# ------------------------------------------------------------------------------
# Unit Tests
# ------------------------------------------------------------------------------

if(BUILD_TESTING)

  enable_testing()
  add_subdirectory(test)

endif(BUILD_TESTING)
