# Ceres tests

manif_add_gtest(gtest_rn_ceres gtest_rn_ceres.cpp)

manif_add_gtest(gtest_so2_ceres gtest_so2_ceres.cpp)

manif_add_gtest(gtest_se2_ceres_autodiff gtest_se2_ceres_autodiff.cpp)
manif_add_gtest(gtest_se2_ceres gtest_se2_ceres.cpp)

manif_add_gtest(gtest_so3_ceres gtest_so3_ceres.cpp)

manif_add_gtest(gtest_se3_ceres gtest_se3_ceres.cpp)

manif_add_gtest(gtest_se23_ceres gtest_se23_ceres.cpp)

manif_add_gtest(gtest_sgal3_ceres gtest_sgal3_ceres.cpp)

manif_add_gtest(gtest_bundle_ceres gtest_bundle_ceres.cpp)

set(CXX_TEST_TARGETS_CERES
  # Rn
  gtest_rn_ceres

  # SO2
  gtest_so2_ceres

  # SO3
  gtest_so3_ceres

  # SE2
  gtest_se2_ceres_autodiff
  gtest_se2_ceres

  # SE3
  gtest_se3_ceres

  # SE23
  gtest_se23_ceres

  # SGal3
  gtest_sgal3_ceres

  # Bundle
  gtest_bundle_ceres
)

foreach(target ${CXX_TEST_TARGETS_CERES})
  target_link_libraries(${target} ${CERES_LIBRARIES})
  target_include_directories(${target} SYSTEM PRIVATE ${CERES_INCLUDE_DIRS})
endforeach()

set(CXX_TEST_TARGETS
  ${CXX_TEST_TARGETS}
  ${CXX_TEST_TARGETS_CERES}

  PARENT_SCOPE
)
