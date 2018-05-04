#include <gtest/gtest.h>

#include "manif/SE2.h"

using namespace manif;

TEST(TEST_SE2, TEST_SE2_TANGENT_0)
{
  double data[3] = {4,2,M_PI};
  Eigen::Map<SE2Tangentd> so2tan(data);

  EXPECT_DOUBLE_EQ(4, so2tan.x());
  EXPECT_DOUBLE_EQ(2, so2tan.y());
  EXPECT_DOUBLE_EQ(M_PI, so2tan.angle());
}

TEST(TEST_SE2, TEST_SE2_TANGENT_DATA)
{
  /// @todo without specifying const
  /// it calls non-const data()

  double data[3] = {4,2,M_PI};
  Eigen::Map<SE2Tangentd> so2tan(data);

  EXPECT_NE(nullptr, so2tan.data());
  EXPECT_EQ(data, so2tan.data());
}

TEST(TEST_SE2, TEST_SE2_TANGENT_ZERO)
{
  double data[3] = {4,2,M_PI};
  Eigen::Map<SE2Tangentd> so2tan(data);

  so2tan.zero();

  EXPECT_DOUBLE_EQ(0, so2tan.x());
  EXPECT_DOUBLE_EQ(0, so2tan.y());
  EXPECT_DOUBLE_EQ(0, so2tan.angle());
}

TEST(TEST_SE2, TEST_SE2_TANGENT_ZERO2)
{
  double data[3] = {4,2,M_PI};
  Eigen::Map<SE2Tangentd> so2tan(data);
  so2tan = SE2Tangentd::Zero();

  EXPECT_DOUBLE_EQ(0, so2tan.x());
  EXPECT_DOUBLE_EQ(0, so2tan.y());
  EXPECT_DOUBLE_EQ(0, so2tan.angle());
}

//TEST(TEST_SE2, TEST_SE2_RANDOM)
//{
//  SE2d so2;

//  so2.random();

//  EXPECT_DOUBLE_EQ(0, so2.angle());
//}

//TEST(TEST_SE2, TEST_SE2_RANDOM2)
//{
//  SE2d so2 = SE2d::Random();

//  EXPECT_DOUBLE_EQ(0, so2.angle());
//}

TEST(TEST_SE2, TEST_SE2_TANGENT_RETRACT)
{
  double data[3] = {4,2,M_PI};
  Eigen::Map<SE2Tangentd> so2tan(data);

  EXPECT_DOUBLE_EQ(4, so2tan.x());
  EXPECT_DOUBLE_EQ(2, so2tan.y());
  EXPECT_DOUBLE_EQ(M_PI, so2tan.angle());

  auto so2_retract = so2tan.retract();

  EXPECT_DOUBLE_EQ(std::cos(M_PI), so2_retract.real());
  EXPECT_DOUBLE_EQ(std::sin(M_PI), so2_retract.imag());
  EXPECT_DOUBLE_EQ(M_PI, so2_retract.angle());

  /// @todo what to expect ? :S
//  EXPECT_DOUBLE_EQ(0, so2_retract.x());
//  EXPECT_DOUBLE_EQ(0, so2_retract.y());
}

/// with Jacs

TEST(TEST_SE2, TEST_SE2_TANGENT_RETRACT_JAC)
{
  double data[3] = {4,2,M_PI};
  Eigen::Map<SE2Tangentd> so2tan(data);

  EXPECT_DOUBLE_EQ(4, so2tan.x());
  EXPECT_DOUBLE_EQ(2, so2tan.y());
  EXPECT_DOUBLE_EQ(M_PI, so2tan.angle());

  SE2d so2_retract;
  SE2d::Jacobian J_ret;

  so2tan.retract(so2_retract, J_ret);

  EXPECT_DOUBLE_EQ(std::cos(M_PI), so2_retract.real());
  EXPECT_DOUBLE_EQ(std::sin(M_PI), so2_retract.imag());
  EXPECT_DOUBLE_EQ(M_PI, so2_retract.angle());

  /// @todo what to expect ? :S
//  EXPECT_DOUBLE_EQ(0, so2_retract.x());
//  EXPECT_DOUBLE_EQ(0, so2_retract.y());

  /// @todo check this J
  EXPECT_EQ(3, J_ret.rows());
  EXPECT_EQ(3, J_ret.cols());
//  EXPECT_DOUBLE_EQ(1, J_ret(0));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
