# SGal3 tests

manif_add_gtest(gtest_sgal3 gtest_sgal3.cpp)

set(CXX_TEST_TARGETS

  ${CXX_TEST_TARGETS}

  # SGal3
  gtest_sgal3

  PARENT_SCOPE
)
