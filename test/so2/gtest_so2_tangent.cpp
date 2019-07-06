#include <gtest/gtest.h>

#include "manif/SO2.h"

using namespace manif;

TEST(TEST_SO2, TEST_SO2_TANGENT_0)
{
  SO2Tangentd so2tan(SO2Tangentd::DataType(MANIF_PI));

  EXPECT_DOUBLE_EQ(MANIF_PI, so2tan.angle());
}

TEST(TEST_SO2, TEST_SO2_TANGENT_1)
{
  SO2Tangentd so2tan(MANIF_PI);

  EXPECT_DOUBLE_EQ(MANIF_PI, so2tan.angle());
}

TEST(TEST_SO2, TEST_SO2_TANGENT_DATA)
{
  const SO2Tangentd so2tan(MANIF_PI);

  EXPECT_DOUBLE_EQ(MANIF_PI, so2tan.coeffs()(0));
}

TEST(TEST_SO2, TEST_SO2_TANGENT_ZERO)
{
  SO2Tangentd so2tan;

  so2tan.setZero();

  EXPECT_DOUBLE_EQ(0, so2tan.angle());
}

TEST(TEST_SO2, TEST_SO2_TANGENT_ZERO2)
{
  SO2Tangentd so2tan = SO2Tangentd::Zero();

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

TEST(TEST_SO2, TEST_SO2_TANGENT_RETRACT)
{
  SO2Tangentd so2_tan(MANIF_PI);

  EXPECT_DOUBLE_EQ(MANIF_PI, so2_tan.angle());

  auto so2_exp = so2_tan.exp();

  EXPECT_DOUBLE_EQ(std::cos(MANIF_PI), so2_exp.real());
  EXPECT_DOUBLE_EQ(std::sin(MANIF_PI), so2_exp.imag());
  EXPECT_DOUBLE_EQ(MANIF_PI, so2_exp.angle());
}

TEST(TEST_SO2, TEST_SO2_TANGENT_SKEW)
{
  SO2Tangentd so2_tan(MANIF_PI);

  EXPECT_DOUBLE_EQ(MANIF_PI, so2_tan.angle());

  SO2Tangentd::LieAlg so2_lie = so2_tan.hat();

  EXPECT_DOUBLE_EQ( 0,    so2_lie(0,0));
  EXPECT_DOUBLE_EQ(-MANIF_PI, so2_lie(0,1));
  EXPECT_DOUBLE_EQ( MANIF_PI, so2_lie(1,0));
  EXPECT_DOUBLE_EQ( 0,    so2_lie(1,1));
}

/// with Jacs

//TEST(TEST_SO2, TEST_SO2_TANGENT_RETRACT_JAC)
//{
//  SO2Tangentd so2_tan(MANIF_PI);

//  EXPECT_DOUBLE_EQ(MANIF_PI, so2_tan.angle());

//  SO2d so2_exp;
//  SO2d::Jacobian J_ret;

//  so2_tan.exp(so2_exp, J_ret);

//  EXPECT_DOUBLE_EQ(std::cos(MANIF_PI), so2_exp.real());
//  EXPECT_DOUBLE_EQ(std::sin(MANIF_PI), so2_exp.imag());
//  EXPECT_DOUBLE_EQ(MANIF_PI, so2_exp.angle());

//  /// @todo check this J
//  EXPECT_EQ(1, J_ret.rows());
//  EXPECT_EQ(1, J_ret.cols());
//  EXPECT_DOUBLE_EQ(1, J_ret(0));
//}

TEST(TEST_SO2, TEST_SO2_TANGENT_RETRACT_OPTJAC)
{
  SO2Tangentd so2_tan(MANIF_PI);

  EXPECT_DOUBLE_EQ(MANIF_PI, so2_tan.angle());

  SO2d so2_exp;
  SO2d::Jacobian J_ret;

  so2_exp = so2_tan.exp(J_ret);

  EXPECT_DOUBLE_EQ(std::cos(MANIF_PI), so2_exp.real());
  EXPECT_DOUBLE_EQ(std::sin(MANIF_PI), so2_exp.imag());
  EXPECT_DOUBLE_EQ(MANIF_PI, so2_exp.angle());

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
