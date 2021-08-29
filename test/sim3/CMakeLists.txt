# Sim3 tests

manif_add_gtest(gtest_sim3 gtest_sim3.cpp)

set(CXX_11_TEST_TARGETS

  ${CXX_11_TEST_TARGETS}

  # Sim3
  gtest_sim3

  PARENT_SCOPE
)
