# Bundle tests

manif_add_gtest(gtest_bundle gtest_bundle.cpp)

set(CXX_11_TEST_TARGETS

  ${CXX_11_TEST_TARGETS}

  gtest_bundle

  PARENT_SCOPE
)
