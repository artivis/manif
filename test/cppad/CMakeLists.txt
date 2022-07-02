# cppad tests

manif_add_gtest(gtest_cppad gtest_cppad.cpp)

set(CXX_11_TEST_TARGETS_CPPAD
  gtest_cppad
)

foreach(target ${CXX_11_TEST_TARGETS_CPPAD})
  target_include_directories(${target} SYSTEM PRIVATE ${CPPAD_INCLUDE_DIR})
endforeach()

set(CXX_11_TEST_TARGETS
  ${CXX_11_TEST_TARGETS}
  ${CXX_11_TEST_TARGETS_CPPAD}

  PARENT_SCOPE
)
