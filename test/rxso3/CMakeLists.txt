# RxSO3 tests

manif_add_gtest(gtest_rxso3 gtest_rxso3.cpp)

set(CXX_11_TEST_TARGETS

  ${CXX_11_TEST_TARGETS}

  # RxSO3
  gtest_rxso3

  PARENT_SCOPE
)
