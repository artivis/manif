# DHu tests

manif_add_gtest(gtest_dhu gtest_dhu.cpp)

set(CXX_11_TEST_TARGETS

  ${CXX_11_TEST_TARGETS}

  # DHu
  gtest_dhu

  PARENT_SCOPE
)
