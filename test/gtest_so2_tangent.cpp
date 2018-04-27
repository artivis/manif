#include <gtest/gtest.h>

#include "manif/SO2.h"

using namespace manif;

TEST(TEST_SO2, TEST_SO2_TANGENT_0)
{
  SO2Tangentd so2tan(SO2Tangentd::TangentDataType(M_PI));

  EXPECT_DOUBLE_EQ(M_PI, so2tan.angle());
}

TEST(TEST_SO2, TEST_SO2_TANGENT_1)
{
  SO2Tangentd so2tan(M_PI);

  EXPECT_DOUBLE_EQ(M_PI, so2tan.angle());
}

TEST(TEST_SO2, TEST_SO2_TANGENT_DATA)
{
  /// @todo without specifying const
  /// it calls non-const data()
  const SO2Tangentd so2tan(M_PI);

  EXPECT_NE(nullptr, so2tan.data());

  EXPECT_DOUBLE_EQ(M_PI, (*so2tan.data())(0));
}

TEST(TEST_SO2, TEST_SO2_TANGENT_ZERO)
{
  SO2Tangentd so2tan;

  so2tan.zero();

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

//  so2.random();

//  EXPECT_DOUBLE_EQ(0, so2.angle());
//}

//TEST(TEST_SO2, TEST_SO2_RANDOM2)
//{
//  SO2d so2 = SO2d::Random();

//  EXPECT_DOUBLE_EQ(0, so2.angle());
//}

TEST(TEST_SO2, TEST_SO2_TANGENT_RETRACT)
{
  SO2Tangentd so2_tan(M_PI);

  EXPECT_DOUBLE_EQ(M_PI, so2_tan.angle());

  auto so2_retract = so2_tan.retract();

  EXPECT_DOUBLE_EQ(std::cos(M_PI), so2_retract.real());
  EXPECT_DOUBLE_EQ(std::sin(M_PI), so2_retract.imag());
  EXPECT_DOUBLE_EQ(M_PI, so2_retract.angle());
}

/// with Jacs

TEST(TEST_SO2, TEST_SO2_TANGENT_RETRACT_JAC)
{
  SO2Tangentd so2_tan(M_PI);

  EXPECT_DOUBLE_EQ(M_PI, so2_tan.angle());

  SO2d so2_retract;
  SO2d::Jacobian J_ret;

  so2_tan.retract(so2_retract, J_ret);

  EXPECT_DOUBLE_EQ(std::cos(M_PI), so2_retract.real());
  EXPECT_DOUBLE_EQ(std::sin(M_PI), so2_retract.imag());
  EXPECT_DOUBLE_EQ(M_PI, so2_retract.angle());

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
