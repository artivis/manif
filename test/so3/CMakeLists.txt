# SO3 tests

manif_add_gtest(gtest_so3 gtest_so3.cpp)

set(CXX_11_TEST_TARGETS

  ${CXX_11_TEST_TARGETS}

  # SO3
  gtest_so3

  PARENT_SCOPE
)
