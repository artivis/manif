cmake_minimum_required(VERSION 3.5.1)

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
endif()

# small helper function
function(manif_add_gtest target)
  add_executable(${target} ${ARGN})
  target_link_libraries(${target} ${PROJECT_NAME} GTest::gtest)

  if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
    # GCC is not strict enough by default, so enable most of the warnings.
    target_compile_options(${target} PRIVATE
      -Werror=all
      -Werror=extra
      )
  endif()

  add_test(NAME ${target} COMMAND ${target})
endfunction()

include_directories(${GTEST_INCLUDE_DIRS})

manif_add_gtest(gtest_misc gtest_misc.cpp)

set(CXX_TEST_TARGETS

  ${CXX_TEST_TARGETS}

  gtest_misc
)

# # R^n tests
add_subdirectory(rn)

# # SO2 tests
add_subdirectory(so2)

# # SO3 tests
add_subdirectory(so3)

# # SE2 tests
add_subdirectory(se2)

# # SE3 tests
add_subdirectory(se3)

# # SE_2_3 tests
add_subdirectory(se_2_3)

# # SGal3 tests
add_subdirectory(sgal3)

if(NOT MSVC)
  # Bundle tests
  add_subdirectory(bundle)
endif()

find_package(Ceres QUIET)

if(CERES_FOUND)

  message(STATUS "Found ceres_solver, building ceres tests.")
  add_subdirectory(ceres)

else()
  message(STATUS "Could not find ceres_solver, ceres tests will not be built.")
endif()

find_package(autodiff QUIET)

if(autodiff_FOUND)

  message(STATUS "Found autodiff, building autodiff tests.")
  add_subdirectory(autodiff)

else()
  message(STATUS "Could not find autodiff, autodiff tests will not be built.")
endif()

if(NOT DEFINED CMAKE_CXX_STANDARD)
  # Set required C++11 flag
  set_property(TARGET ${CXX_TEST_TARGETS} PROPERTY CXX_STANDARD 11)
endif()
set_property(TARGET ${CXX_TEST_TARGETS} PROPERTY CXX_STANDARD_REQUIRED ON)
set_property(TARGET ${CXX_TEST_TARGETS} PROPERTY CXX_EXTENSIONS OFF)
