cmake_minimum_required(VERSION 2.8.3)
project(manif CXX)

# Check that the compoiler has c++11 support
# Crash otherwise.
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
if(COMPILER_SUPPORTS_CXX11)
  message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has C++11 support.")
else()
  message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
  add_compile_options(-ftemplate-depth=1024)
elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
  # GCC is not strict enough by default, so enable most of the warnings.
  #add_compile_options(-Werror=all)
  #add_compile_options(-Werror=extra)
  #add_compile_options(-Wno-unknown-pragmas)
  #add_compile_options(-Wno-sign-compare)
  #add_compile_options(-Wno-unused-parameter)
  #add_compile_options(-Wno-missing-field-initializers)
endif()

find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)
find_package(Ceres QUIET)

add_subdirectory(external/lspdlog)

# Options. Turn on with 'cmake -DBUILD_TESTING=ON'.
# catkin build manif --cmake-args -DBUILD_TESTING=ON
option(BUILD_TESTING "Build all tests." OFF)

###########
## Build ##
###########

#include_directories(include)
#include_directories(external/lt)
include_directories(SYSTEM
  ${EIGEN_INCLUDE_DIRS}
  ${LSPDLOG_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} INTERFACE)
#target_include_directories(${PROJECT_NAME} INTERFACE include)
#target_include_directories(${PROJECT_NAME} INTERFACE external/lt)
target_compile_definitions(${PROJECT_NAME} INTERFACE -std=c++11)
#set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 11)

target_include_directories(${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/external/lt>
  $<INSTALL_INTERFACE:include>
)

#############
## Install ##
#############

set(config_install_dir "lib/cmake/${PROJECT_NAME}")
set(include_install_dir "include")

set(generated_dir "${CMAKE_CURRENT_BINARY_DIR}/generated")

# Configuration
set(version_config "${generated_dir}/${PROJECT_NAME}ConfigVersion.cmake")
set(project_config "${generated_dir}/${PROJECT_NAME}Config.cmake")
set(namespace "${PROJECT_NAME}::")

include(CMakePackageConfigHelpers)

# Configure '<PROJECT-NAME>ConfigVersion.cmake'
write_basic_package_version_file(
  ${PROJECT_NAME}ConfigVersion.cmake
  VERSION 0.0.0
  COMPATIBILITY AnyNewerVersion
)

#message(WARNING ${${PROJECT_NAME}Targets})

configure_package_config_file(
  "${PROJECT_SOURCE_DIR}/cmake/${PROJECT_NAME}Config.cmake.in"
  "${project_config}"
  INSTALL_DESTINATION "${config_install_dir}"
)

# Targets:
install(
  TARGETS  ${PROJECT_NAME}
  EXPORT   ${PROJECT_NAME}Targets
  LIBRARY  DESTINATION "lib" #COMPONENT Runtime
  ARCHIVE  DESTINATION "lib" #COMPONENT Development
  RUNTIME  DESTINATION "bin" #COMPONENT Runtime
  INCLUDES DESTINATION "${include_install_dir}"
)

# Headers:
install(
  DIRECTORY
    ${PROJECT_SOURCE_DIR}/include/
    ${PROJECT_SOURCE_DIR}/external/lt/
  DESTINATION include
)

# Export headers:
#install(
#  FILES "${bar_export}" "${baz_export}"
#  DESTINATION "${include_install_dir}/foo"
#)

# Config
install(
  FILES
    "${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
    "${project_config}"
  DESTINATION
    "${config_install_dir}"
)

install(
  EXPORT ${PROJECT_NAME}Targets
  FILE ${PROJECT_NAME}Targets.cmake
#  NAMESPACE MANIF::
  DESTINATION
    "${config_install_dir}"
)





#########################################################

#message(WARNING ${CMAKE_INSTALL_PREFIX})

# Mark cpp header files for installation
#install(DIRECTORY include/${PROJECT_NAME}
#  DESTINATION include
#  FILES_MATCHING PATTERN "*.h"
#  PATTERN ".svn" EXCLUDE
#)

install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

###############
## Packaging ##
###############

#target_include_directories(
#  PUBLIC
#  $<INSTALL_INTERFACE:include>
#  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#)

#############
## Testing ##
#############

if(BUILD_TESTING)

  include_directories(SYSTEM
    ${CERES_INCLUDE_DIRS}
  )

  enable_testing()
  add_subdirectory(test)

endif(BUILD_TESTING)
