#include "manif/SE2.h"

#include "../gtest_manif_utils.h"

using namespace manif;

TEST(TEST_SE2, TEST_SE2_TANGENT_0)
{
  SE2Tangentd so2tan(4,2,MANIF_PI);

  EXPECT_DOUBLE_EQ(4, so2tan.x());
  EXPECT_DOUBLE_EQ(2, so2tan.y());
  EXPECT_DOUBLE_EQ(MANIF_PI, so2tan.angle());
}

TEST(TEST_SE2, TEST_SE2_TANGENT_DATA)
{
  SE2Tangentd so2tan(4,2,MANIF_PI);

  EXPECT_NE(nullptr, so2tan.data());
}

TEST(TEST_SE2, TEST_SE2_TANGENT_ZERO)
{
  SE2Tangentd so2tan(4,2,MANIF_PI);

  so2tan.setZero();

  EXPECT_DOUBLE_EQ(0, so2tan.x());
  EXPECT_DOUBLE_EQ(0, so2tan.y());
  EXPECT_DOUBLE_EQ(0, so2tan.angle());
}

TEST(TEST_SE2, TEST_SE2_TANGENT_ZERO2)
{
  SE2Tangentd so2tan(4,2,MANIF_PI);
  so2tan = SE2Tangentd::Zero();

  EXPECT_DOUBLE_EQ(0, so2tan.x());
  EXPECT_DOUBLE_EQ(0, so2tan.y());
  EXPECT_DOUBLE_EQ(0, so2tan.angle());
}

//TEST(TEST_SE2, TEST_SE2_RANDOM)
//{
//  SE2d so2;

//  so2.setRandom();

//  EXPECT_DOUBLE_EQ(0, so2.angle());
//}

//TEST(TEST_SE2, TEST_SE2_RANDOM2)
//{
//  SE2d so2 = SE2d::Random();

//  EXPECT_DOUBLE_EQ(0, so2.angle());
//}

TEST(TEST_SE2, TEST_SE2_TANGENT_RETRACT)
{
  SE2Tangentd so2tan(4,2,MANIF_PI);

  EXPECT_DOUBLE_EQ(4, so2tan.x());
  EXPECT_DOUBLE_EQ(2, so2tan.y());
  EXPECT_DOUBLE_EQ(MANIF_PI, so2tan.angle());

  auto so2_exp = so2tan.exp();

  EXPECT_DOUBLE_EQ(std::cos(MANIF_PI), so2_exp.real());
  EXPECT_DOUBLE_EQ(std::sin(MANIF_PI), so2_exp.imag());
  EXPECT_DOUBLE_EQ(MANIF_PI, so2_exp.angle());

  /// @todo what to expect ? :S
//  EXPECT_DOUBLE_EQ(0, so2_exp.x());
//  EXPECT_DOUBLE_EQ(0, so2_exp.y());
}

/// with Jacs

TEST(TEST_SE2, TEST_SE2_TANGENT_RETRACT_JAC)
{
  SE2Tangentd so2tan(4,2,MANIF_PI);

  EXPECT_DOUBLE_EQ(4, so2tan.x());
  EXPECT_DOUBLE_EQ(2, so2tan.y());
  EXPECT_DOUBLE_EQ(MANIF_PI, so2tan.angle());

  SE2d::Jacobian J_ret;
  SE2d so2_exp = so2tan.exp(J_ret);

  EXPECT_DOUBLE_EQ(std::cos(MANIF_PI), so2_exp.real());
  EXPECT_DOUBLE_EQ(std::sin(MANIF_PI), so2_exp.imag());
  EXPECT_DOUBLE_EQ(MANIF_PI, so2_exp.angle());

  /// @todo what to expect ? :S
//  EXPECT_DOUBLE_EQ(0, so2_exp.x());
//  EXPECT_DOUBLE_EQ(0, so2_exp.y());

  /// @todo check this J
  EXPECT_EQ(3, J_ret.rows());
  EXPECT_EQ(3, J_ret.cols());
//  EXPECT_DOUBLE_EQ(1, J_ret(0));
}

TEST(TEST_SE2, TEST_SE2_TANGENT_INSTREAM)
{
  SE2Tangentd se2;

  se2 << 4, 2, MANIF_PI;

  EXPECT_DOUBLE_EQ(4,    se2.x());
  EXPECT_DOUBLE_EQ(2,    se2.y());
  EXPECT_DOUBLE_EQ(MANIF_PI, se2.angle());

  se2 << 3.5, 3.5, MANIF_PI_4;

  EXPECT_DOUBLE_EQ(3.5,    se2.x());
  EXPECT_DOUBLE_EQ(3.5,    se2.y());
  EXPECT_DOUBLE_EQ(MANIF_PI_4, se2.angle());

  typename SE2Tangentd::DataType data(1,2,3);
  se2 << data;

  EXPECT_DOUBLE_EQ(1, se2.x());
  EXPECT_DOUBLE_EQ(2, se2.y());
  EXPECT_DOUBLE_EQ(3, se2.angle());
}

TEST(TEST_SE2, TEST_SE2_TANGENT_SMALL_ANGLE_JACOBIANS)
{
  // Angles on both sides of the small angle thresholds
  const double angles[] = {
    0, 1e-8, 1e-7, 1.4e-7, 1.6e-7, 1e-6, 1e-5, 1e-4, 1e-3,
    1e-2, 1.9e-2, 2e-2, 1e-1, 0.5
  };

  for (const double angle : angles)
  {
    for (const double sign : {1., -1.})
    {
      const SE2Tangentd tan(0.5, -0.8, sign * angle);
      const JacobiansSeries<SE2Tangentd> series(tan);

      SCOPED_TRACE(::testing::Message() << "angle=" << tan.angle());

      EXPECT_EIGEN_NEAR(series.Jr, tan.rjac(), 1e-14);
      EXPECT_EIGEN_NEAR(series.Jl, tan.ljac(), 1e-14);
      EXPECT_EIGEN_NEAR(series.Jrinv, tan.rjacinv(), 1e-14);
      EXPECT_EIGEN_NEAR(series.Jlinv, tan.ljacinv(), 1e-14);

      SE2d::Jacobian J_exp, J_log;
      const SE2d state = tan.exp(J_exp);
      const SE2Tangentd tan_log = state.log(J_log);

      EXPECT_EIGEN_NEAR(series.Jr, J_exp, 1e-14);
      EXPECT_EIGEN_NEAR(series.Jrinv, J_log, 1e-14);
      EXPECT_EIGEN_NEAR(tan.coeffs(), tan_log.coeffs(), 1e-14);
    }
  }
}

MANIF_RUN_ALL_TEST;
