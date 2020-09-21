cmake_minimum_required(VERSION 2.8.8)

find_package(GTest QUIET)

if(NOT GTEST_FOUND)
  # If not found, download it

  # Download and unpack googletest at configure time
  configure_file(gtest/CMakeLists.txt.in gtest/CMakeLists.txt)
  execute_process(COMMAND ${CMAKE_COMMAND} -G "${CMAKE_GENERATOR}" .
    RESULT_VARIABLE result
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/gtest )
  if(result)
    message(FATAL_ERROR "CMake step for googletest failed: ${result}")
  endif()
  execute_process(COMMAND ${CMAKE_COMMAND} --build .
    RESULT_VARIABLE result
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/gtest )
  if(result)
    message(FATAL_ERROR "Build step for googletest failed: ${result}")
  endif()

  # Prevent overriding the parent project's compiler/linker
  # settings on Windows
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

  # Add googletest directly to our build. This defines
  # the gtest and gtest_main targets.
  add_subdirectory(${CMAKE_CURRENT_BINARY_DIR}/googletest-src
                   ${CMAKE_CURRENT_BINARY_DIR}/googletest-build
                   EXCLUDE_FROM_ALL)

  # The gtest/gtest_main targets carry header search path
  # dependencies automatically when using CMake 2.8.11 or
  # later. Otherwise we have to add them here ourselves.
  if (CMAKE_VERSION VERSION_LESS 2.8.11)
    include_directories("${gtest_SOURCE_DIR}/include")
  endif()
endif()

# small helper function
function(manif_add_gtest target)
  add_executable(${target} ${ARGN})
  add_dependencies(${target} gtest)
  target_link_libraries(${target} ${PROJECT_NAME} gtest)

  if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
    # GCC is not strict enough by default, so enable most of the warnings.
    target_compile_options(${target} PRIVATE
      -Werror=all
      -Werror=extra
      -Wno-unknown-pragmas
      -Wno-sign-compare
      -Wno-unused-parameter
      -Wno-missing-field-initializers
      )
  endif()

  add_test(NAME ${target} COMMAND ${target})
endfunction()

include_directories(${GTEST_INCLUDE_DIRS})

# R^n tests
add_subdirectory(rn)

# SO2 tests
add_subdirectory(so2)

# SO3 tests
add_subdirectory(so3)

# SE2 tests
add_subdirectory(se2)

# SE3 tests
add_subdirectory(se3)

# DHu tests
add_subdirectory(dhu)

find_package(Ceres QUIET)

if(CERES_FOUND)

  message(STATUS "Found ceres_solver, building ceres tests.")
  add_subdirectory(ceres)

else()
  message(STATUS "Could not find ceres_solver, ceres tests will not be built.")
endif()

# Set required C++11 flag
if(CMAKE_VERSION VERSION_LESS "3.1")
  set_target_properties(${CXX_11_TEST_TARGETS} PROPERTIES COMPILE_FLAGS "-std=c++11")
else()
  set_property(TARGET ${CXX_11_TEST_TARGETS} PROPERTY CXX_STANDARD 11)
  set_property(TARGET ${CXX_11_TEST_TARGETS} PROPERTY CXX_STANDARD_REQUIRED ON)
  set_property(TARGET ${CXX_11_TEST_TARGETS} PROPERTY CXX_EXTENSIONS OFF)
endif()
