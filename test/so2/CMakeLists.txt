# SO2 tests

# so2 tests
manif_add_gtest(gtest_so2 gtest_so2.cpp)

# so2 throuhg base template tests
manif_add_gtest(gtest_so2_base_template gtest_so2_base_template.cpp)

# so2 Eigen::Map tests
manif_add_gtest(gtest_so2_map gtest_so2_map.cpp)

# so2 tangent tests
manif_add_gtest(gtest_so2_tangent gtest_so2_tangent.cpp)

# so2 tangent throuhg base template tests
manif_add_gtest(gtest_so2_tangent_base_template
  gtest_so2_tangent_base_template.cpp)

# so2 tangent Eigen::Map tests
manif_add_gtest(gtest_so2_tangent_map gtest_so2_tangent_map.cpp)

set(CXX_11_TEST_TARGETS

  ${CXX_11_TEST_TARGETS}

  # SO2
  gtest_so2
  gtest_so2_base_template
  gtest_so2_map
  gtest_so2_tangent
  gtest_so2_tangent_base_template
  gtest_so2_tangent_map

  PARENT_SCOPE
)
