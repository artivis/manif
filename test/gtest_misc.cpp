#include <gtest/gtest.h>

#include "manif/manif.h"
#include "gtest_manif_utils.h"
#include "gtest_eigen_utils.h"

using namespace manif;

TEST(TEST_MISC, TEST_SKEW)
{
  constexpr double s = 1.5;

  EXPECT_EIGEN_NEAR((Eigen::Matrix2d() << 0., -s, s, 0.).finished(), skew(s));
}

TEST(TEST_MISC, TEST_SKEW3)
{
  EXPECT_EIGEN_NEAR(
    (Eigen::Matrix3d() <<
            0.,  -3.7,   +2.6,
            +3.7,   0.,  -1.5,
            -2.6,   +1.5,   0.).finished(),
    skew(Eigen::Vector3d(1.5, 2.6, 3.7))
  );
}

TEST(TEST_MISC, TEST_SKEWd)
{
  Eigen::VectorXd s(1, 1);
  s << 1.5;

  EXPECT_EIGEN_NEAR((Eigen::Matrix2d() << 0., -1.5, 1.5, 0.).finished(), skew(s));

  s.resize(3, 1);
  s << 1.5, 2.6, 3.7;

  EXPECT_EIGEN_NEAR((Eigen::Matrix3d() <<
            0.,  -3.7,   +2.6,
            +3.7,   0.,  -1.5,
            -2.6,   +1.5,   0.).finished(), skew(s));

  s.resize(5, 1);
  s << 1.5, 2.6, 3.7, 1., 1.;

  EXPECT_THROW(skew(s), manif::runtime_error);
}

MANIF_RUN_ALL_TEST;
