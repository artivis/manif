#include <gtest/gtest.h>

#include "manif/SO2.h"

using namespace manif;

TEST(TEST_SO2, TEST_SO2_0)
{
  SO2d so2(SO2d::ManifoldDataType(1,0));

  EXPECT_DOUBLE_EQ(0, so2.angle());
}

TEST(TEST_SO2, TEST_SO2_1)
{
  SO2d so2(1, 0);

  EXPECT_DOUBLE_EQ(0, so2.angle());
}

TEST(TEST_SO2, TEST_SO2_2)
{
  SO2d so2(0);

  EXPECT_DOUBLE_EQ(0, so2.angle());
}

TEST(TEST_SO2, TEST_SO2_DATA)
{
  /// @todo without specifying const
  /// it calls non-const data()
  const SO2d so2(0);

  EXPECT_NE(nullptr, so2.data());

  EXPECT_DOUBLE_EQ(1, (*so2.data())(0));
  EXPECT_DOUBLE_EQ(0, (*so2.data())(1));
}

TEST(TEST_SO2, TEST_SO2_IDENTITY)
{
  SO2d so2;

  so2.identity();

  EXPECT_DOUBLE_EQ(0, so2.angle());
  EXPECT_DOUBLE_EQ(1, so2.real());
  EXPECT_DOUBLE_EQ(0, so2.imag());
}

TEST(TEST_SO2, TEST_SO2_IDENTITY2)
{
  SO2d so2 = SO2d::Identity();

  EXPECT_DOUBLE_EQ(0, so2.angle());
  EXPECT_DOUBLE_EQ(1, so2.real());
  EXPECT_DOUBLE_EQ(0, so2.imag());
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

TEST(TEST_SO2, TEST_SO2_MATRIX)
{
  SO2d so2 = SO2d::Identity();

  SO2d::Transformation t = so2.matrix();

  EXPECT_EQ(3, t.rows());
  EXPECT_EQ(3, t.cols());

  /// @todo Eigen matrix comparison
}

TEST(TEST_SO2, TEST_SO2_ROTATION)
{
  SO2d so2 = SO2d::Identity();

  SO2d::Rotation r = so2.rotation();

  EXPECT_EQ(2, r.rows());
  EXPECT_EQ(2, r.cols());

  /// @todo Eigen matrix comparison
}

TEST(TEST_SO2, TEST_SO2_INVERSE)
{
  SO2d so2 = SO2d::Identity();

  auto so2_inv = so2.inverse();

  EXPECT_DOUBLE_EQ(so2.angle(), so2_inv.angle());
  EXPECT_DOUBLE_EQ(1, so2_inv.real());
  EXPECT_DOUBLE_EQ(0, so2_inv.imag());

  so2.angle(M_PI);
  so2_inv = so2.inverse();

  EXPECT_DOUBLE_EQ(-M_PI, so2_inv.angle());
}

TEST(TEST_SO2, TEST_SO2_RPLUS)
{
  SO2d so2a(M_PI / 2.);
  SO2Tangentd so2b(M_PI / 2.);

  auto so2c = so2a.rplus(so2b);

  /// @todo
//  EXPECT_DOUBLE_EQ(0., so2c.angle());
}

TEST(TEST_SO2, TEST_SO2_LPLUS)
{
  SO2d so2a(M_PI / 2.);
  SO2Tangentd so2b(M_PI / 2.);

  auto so2c = so2a.lplus(so2b);

  /// @todo
//  EXPECT_DOUBLE_EQ(0., so2c.angle());
}

TEST(TEST_SO2, TEST_SO2_PLUS)
{
  SO2d so2a(M_PI / 2.);
  SO2Tangentd so2b(M_PI / 2.);

  auto so2c = so2a.plus(so2b);

  /// @todo
//  EXPECT_DOUBLE_EQ(0., so2c.angle());
}

TEST(TEST_SO2, TEST_SO2_OP_PLUS)
{
  SO2d so2a(M_PI / 2.);
  SO2Tangentd so2b(M_PI / 2.);

  auto so2c = so2a + so2b;

  /// @todo
//  EXPECT_DOUBLE_EQ(0., so2c.angle());
}

TEST(TEST_SO2, TEST_SO2_OP_PLUS_EQ)
{
  SO2d so2a(M_PI / 2.);
  SO2Tangentd so2b(M_PI / 2.);

  so2a +=so2b;

  /// @todo
//  EXPECT_DOUBLE_EQ(0., so2c.angle());
}

TEST(TEST_SO2, TEST_SO2_RMINUS)
{
  SO2d so2a(M_PI);
  SO2d so2b(M_PI_2);

  auto so2c = so2a.rminus(so2b);

  /// @todo
//  EXPECT_DOUBLE_EQ(-M_PI_2, so2c.angle());
}

TEST(TEST_SO2, TEST_SO2_LMINUS)
{
  SO2d so2a(M_PI / 2.);
  SO2d so2b(M_PI / 2.);

  auto so2c = so2a.lminus(so2b);

  /// @todo
//  EXPECT_DOUBLE_EQ(0., so2c.angle());
}

//TEST(TEST_SO2, TEST_SO2_MINUS)
//{
//  SO2d so2a(M_PI / 2.);
//  SO2d so2b(M_PI / 2.);

//  auto so2c = so2a.minus(so2b);

//  /// @todo
////  EXPECT_DOUBLE_EQ(0., so2c.angle());
//}

TEST(TEST_SO2, TEST_SO2_LIFT)
{
  SO2d so2(M_PI);

  auto so2_lift = so2.lift();

  EXPECT_DOUBLE_EQ(M_PI, so2_lift.angle());
}

/// @todo move to SO2Tangent tests
TEST(TEST_SO2, TEST_SO2_RETRACT)
{
  SO2d so2(M_PI);

  auto so2_lift = so2.lift();

  EXPECT_DOUBLE_EQ(M_PI, so2_lift.angle());

  auto so2_retract = so2_lift.retract();

  EXPECT_DOUBLE_EQ(std::cos(M_PI), so2_retract.real());
  EXPECT_DOUBLE_EQ(std::sin(M_PI), so2_retract.imag());
  EXPECT_DOUBLE_EQ(M_PI, so2_retract.angle());
}

TEST(TEST_SO2, TEST_SO2_COMPOSE)
{
  SO2d so2a(M_PI_2);
  SO2d so2b(M_PI_2);

  auto so2c = so2a.compose(so2b);

  EXPECT_DOUBLE_EQ(M_PI, so2c.angle());
}

TEST(TEST_SO2, TEST_SO2_OP_COMPOSE)
{
  SO2d so2a(M_PI_2);
  SO2d so2b(M_PI_2);

  auto so2c = so2a * so2b;

  EXPECT_DOUBLE_EQ(M_PI, so2c.angle());
}

TEST(TEST_SO2, TEST_SO2_OP_COMPOSE_EQ)
{
  SO2d so2a(M_PI_2);
  SO2d so2b(M_PI_2);

  so2a *= so2b;

  EXPECT_DOUBLE_EQ(M_PI, so2a.angle());
}

TEST(TEST_SO2, TEST_SO2_BETWEEN)
{
  SO2d so2a(M_PI);
  SO2d so2b(M_PI_2);

  auto so2c = so2a.between(so2b);

  EXPECT_DOUBLE_EQ(-M_PI_2, so2c.angle());
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
