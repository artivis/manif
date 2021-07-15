# RxSO2 tests

manif_add_gtest(gtest_rxso2 gtest_rxso2.cpp)

set(CXX_11_TEST_TARGETS

  ${CXX_11_TEST_TARGETS}

  # RxSO2
  gtest_rxso2

  PARENT_SCOPE
)
