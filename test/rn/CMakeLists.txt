# SO3 tests

manif_add_gtest(gtest_rn gtest_rn.cpp)

set(CXX_14_TEST_TARGETS

  ${CXX_14_TEST_TARGETS}

  # R^n
  gtest_rn

  PARENT_SCOPE
)
