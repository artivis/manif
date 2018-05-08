# SE2 tests

manif_add_gtest(gtest_se2 gtest_se2.cpp)
manif_add_gtest(gtest_se2_map gtest_se2_map.cpp)
manif_add_gtest(gtest_se2_tangent gtest_se2_tangent.cpp)
manif_add_gtest(gtest_se2_tangent_map gtest_se2_tangent_map.cpp)

set(CXX_11_TEST_TARGETS

  ${CXX_11_TEST_TARGETS}

  # SE2
  gtest_se2
  gtest_se2_map
  gtest_se2_tangent
  gtest_se2_tangent_map

  PARENT_SCOPE
)
