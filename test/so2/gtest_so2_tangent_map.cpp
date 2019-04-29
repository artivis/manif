#include <gtest/gtest.h>

#include "manif/SO2.h"

using namespace manif;

TEST(TEST_SO2, TEST_SO2_TANGENT_MAP_0)
{
  double data(M_PI);
  Eigen::Map<SO2Tangentd> so2tan(&data);

  EXPECT_DOUBLE_EQ(M_PI, so2tan.angle());
}

TEST(TEST_SO2, TEST_SO2_TANGENT_MAP_DATA)
{
  /// @todo without specifying const
  /// it calls non-const data()

  double data(M_PI);
  const Eigen::Map<SO2Tangentd> so2tan(&data);

//  EXPECT_NE(nullptr, so2tan.data());
//  EXPECT_EQ(&data, so2tan.data()->data());

//  EXPECT_DOUBLE_EQ(M_PI, (*so2tan.data())(0));
}

TEST(TEST_SO2, TEST_SO2_TANGENT_MAP_ZERO)
{
  double data(1);
  Eigen::Map<SO2Tangentd> so2tan(&data);

  so2tan.setZero();

  EXPECT_DOUBLE_EQ(0, so2tan.angle());
}

TEST(TEST_SO2, TEST_SO2_TANGENT_MAP_ZERO2)
{
  double data(1);
  Eigen::Map<SO2Tangentd> so2tan(&data);
  so2tan = SO2Tangentd::Zero();

  EXPECT_DOUBLE_EQ(0, so2tan.angle());
}

//TEST(TEST_SO2, TEST_SO2_RANDOM)
//{
//  SO2d so2;

//  so2.setRandom();

//  EXPECT_DOUBLE_EQ(0, so2.angle());
//}

//TEST(TEST_SO2, TEST_SO2_RANDOM2)
//{
//  SO2d so2 = SO2d::Random();

//  EXPECT_DOUBLE_EQ(0, so2.angle());
//}

TEST(TEST_SO2, TEST_SO2_TANGENT_MAP_RETRACT)
{
  double data(M_PI);
  Eigen::Map<SO2Tangentd> so2tan(&data);

  EXPECT_DOUBLE_EQ(M_PI, so2tan.angle());

  auto so2_exp = so2tan.exp();

  EXPECT_DOUBLE_EQ(std::cos(M_PI), so2_exp.real());
  EXPECT_DOUBLE_EQ(std::sin(M_PI), so2_exp.imag());
  EXPECT_DOUBLE_EQ(M_PI, so2_exp.angle());
}

/// with Jacs

TEST(TEST_SO2, TEST_SO2_TANGENT_MAP_RETRACT_JAC)
{
  double data(M_PI);
  Eigen::Map<SO2Tangentd> so2tan(&data);

  EXPECT_DOUBLE_EQ(M_PI, so2tan.angle());

  Jacobian<SO2d,SO2Tangentd> J_ret;

  SO2d so2_exp = so2tan.exp(J_ret);

  EXPECT_DOUBLE_EQ(std::cos(M_PI), so2_exp.real());
  EXPECT_DOUBLE_EQ(std::sin(M_PI), so2_exp.imag());
  EXPECT_DOUBLE_EQ(M_PI, so2_exp.angle());

  /// @todo check this J
  EXPECT_EQ(1, J_ret.rows());
  EXPECT_EQ(1, J_ret.cols());
  EXPECT_DOUBLE_EQ(1, J_ret(0));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
