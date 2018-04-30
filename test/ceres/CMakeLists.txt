# Ceres tests

manif_add_gtest(gtest_so2_local_parametrization
  gtest_so2_local_parametrization.cpp)

#manif_add_gtest(gtest_so3_ceres gtest_so3_ceres.cpp)
#manif_add_gtest(gtest_se2_ceres gtest_se2_ceres.cpp)
#manif_add_gtest(gtest_se3_ceres gtest_se3_ceres.cpp)

set(CXX_11_TEST_TARGETS_CERES
  # SO2
  gtest_so2_local_parametrization

  # SO3
#  gtest_so3_ceres

  # SE2
#  gtest_se2_ceres

  # SE3
#  gtest_se3_ceres
)

foreach(target ${CXX_11_TEST_TARGETS_CERES})
  target_link_libraries(${target} ${ceres_solver_LIBRARIES})
endforeach(test_src)

set(CXX_11_TEST_TARGETS
  ${CXX_11_TEST_TARGETS}
  ${CXX_11_TEST_TARGETS_CERES}
  PARENT_SCOPE
)
