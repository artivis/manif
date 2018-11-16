cmake_minimum_required(VERSION 2.8.8)

find_package(GTest QUIET)

if(NOT GTEST_FOUND)
  # If not found, download it
  add_subdirectory(gtest)
endif()

# small helper function
function(manif_add_gtest target)
  add_executable(${target} ${ARGN})
  add_dependencies(${target} libgtest)
  target_link_libraries(${target} ${PROJECT_NAME} libgtest)

  add_test(NAME ${target} COMMAND ${target})
endfunction()

include_directories(${GTEST_INCLUDE_DIRS})

# SO2 tests
add_subdirectory(so2)

# SO3 tests
add_subdirectory(so3)

# SE2 tests
add_subdirectory(se2)

# SE3 tests
add_subdirectory(se3)

##################
## Ceres Solver ##
##################

find_package(Ceres QUIET)

if(CERES_FOUND)

  message(STATUS "Found ceres_solver, building related tests.")
  add_subdirectory(ceres)

else()
  message(STATUS "Could not find ceres_solver, related tests will not be built.")
endif()

###########
## CppAD ##
###########

#find_package(CppAD QUIET)

#if(CppAD_FOUND)

#  message(STATUS "Found CppAD, building related tests.")
#  add_subdirectory(cppad)

#else()
#  message(STATUS "Could not find CppAD, related tests will not be built.")
#endif()

# Set required C++11 flag
if(CMAKE_VERSION VERSION_LESS "3.1")
  set_target_properties(${CXX_11_TEST_TARGETS} PROPERTIES COMPILE_FLAGS "-std=c++11")
else()
  set_property(TARGET ${CXX_11_TEST_TARGETS} PROPERTY CXX_STANDARD 11)
  set_property(TARGET ${CXX_11_TEST_TARGETS} PROPERTY CXX_STANDARD_REQUIRED ON)
  set_property(TARGET ${CXX_11_TEST_TARGETS} PROPERTY CXX_EXTENSIONS OFF)
endif()
