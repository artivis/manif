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

find_package(Ceres REQUIRED)

add_subdirectory(external/lspdlog)

# Options. Turn on with 'cmake -DBUILD_TESTING=ON'.
# catkin build manif --cmake-args -DBUILD_TESTING=ON
option(BUILD_TESTING "Build all tests." OFF)

###################################
## catkin specific configuration ##
###################################
#catkin_package(
#  INCLUDE_DIRS include #${EIGEN_INCLUDE_DIRS}
##  LIBRARIES ${PROJECT_NAME}
##  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS Eigen
#)

###########
## Build ##
###########

include_directories(include)
include_directories(external/lt)
include_directories(SYSTEM
  ${EIGEN_INCLUDE_DIRS}
  ${LSPDLOG_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
)

#############
## Install ##
#############

#message(WARNING ${CMAKE_INSTALL_PREFIX})

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

#############
## Testing ##
#############

if(BUILD_TESTING)

  enable_testing()
  add_subdirectory(test)

endif(BUILD_TESTING)
