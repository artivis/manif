# Bundle tests

manif_add_gtest(gtest_bundle gtest_bundle.cpp)
manif_add_gtest(gtest_bundle_single_group gtest_bundle_single_group.cpp)
manif_add_gtest(gtest_bundle_large gtest_bundle_large.cpp)

set(CXX_14_TEST_TARGETS

  ${CXX_14_TEST_TARGETS}

  gtest_bundle
  gtest_bundle_single_group
  gtest_bundle_large

  PARENT_SCOPE
)
