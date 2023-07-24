# SO3 tests

manif_add_gtest(gtest_so3 gtest_so3.cpp)

set(CXX_14_TEST_TARGETS

  ${CXX_14_TEST_TARGETS}

  # SO3
  gtest_so3

  PARENT_SCOPE
)
