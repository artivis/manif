# Sim3 tests

manif_add_gtest(gtest_sim2 gtest_sim2.cpp)

set(CXX_11_TEST_TARGETS

  ${CXX_11_TEST_TARGETS}

  # Sim2
  gtest_sim2

  PARENT_SCOPE
)
