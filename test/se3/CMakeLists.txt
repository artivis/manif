# SE3 tests

manif_add_gtest(gtest_se3 gtest_se3.cpp)

set(CXX_14_TEST_TARGETS

  ${CXX_14_TEST_TARGETS}

  # SE3
  gtest_se3

  PARENT_SCOPE
)
