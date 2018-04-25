cmake_minimum_required(VERSION 2.8.8)
find_package(GTest REQUIRED)

# small helper function
function(manif_add_gtest target)
  add_executable(${target} ${ARGN})
  add_dependencies(${target} libgtest)
  target_link_libraries(${target} libgtest)

  #WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/bin
  add_test(NAME ${target} COMMAND ${target})
endfunction()

include_directories(${GTEST_INCLUDE_DIRS})

manif_add_gtest(gtest_test gtest_test.cpp)

set(CXX_11_TEST_TARGETS
  gtest_test
)

# Set required C++11 flag
if(CMAKE_VERSION VERSION_LESS "3.1")
  set_target_properties(${CXX_11_TEST_TARGETS} PROPERTIES COMPILE_FLAGS "-std=c++11")
else()
  set_property(TARGET ${CXX_11_TEST_TARGETS} PROPERTY CXX_STANDARD 11)
  set_property(TARGET ${CXX_11_TEST_TARGETS} PROPERTY CXX_STANDARD_REQUIRED ON)
  set_property(TARGET ${CXX_11_TEST_TARGETS} PROPERTY CXX_EXTENSIONS OFF)
endif()
