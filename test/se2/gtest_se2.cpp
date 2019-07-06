#include <gtest/gtest.h>

#include "manif/SE2.h"
#include "../common_tester.h"

using namespace manif;

TEST(TEST_SE2, TEST_SE2_CONSTRUCTOR_DATATYPE)
{
  SE2d se2(SE2d::DataType(0,0,1,0));

  EXPECT_DOUBLE_EQ(0, se2.x());
  EXPECT_DOUBLE_EQ(0, se2.y());
  EXPECT_DOUBLE_EQ(0, se2.angle());
}

TEST(TEST_SE2, TEST_SE2_CONSTRUCTOR_X_Y_REAL_IMAG)
{
  SE2d se2(4, 2, 1, 0);

  EXPECT_DOUBLE_EQ(4, se2.x());
  EXPECT_DOUBLE_EQ(2, se2.y());
  EXPECT_DOUBLE_EQ(1, se2.real());
  EXPECT_DOUBLE_EQ(0, se2.imag());
  EXPECT_DOUBLE_EQ(0, se2.angle());
}

TEST(TEST_SE2, TEST_SE2_CONSTRUCTOR_X_Y_THETA)
{
  SE2d se2(4, 2, 0);

  EXPECT_DOUBLE_EQ(4, se2.x());
  EXPECT_DOUBLE_EQ(2, se2.y());
  EXPECT_DOUBLE_EQ(0, se2.angle());
}

TEST(TEST_SE2, TEST_SE2_CONSTRUCTOR_COPY)
{
  SE2d se2(SE2d(4, 2, 1,1));

  EXPECT_DOUBLE_EQ(4, se2.x());
  EXPECT_DOUBLE_EQ(2, se2.y());
  EXPECT_DOUBLE_EQ(MANIF_PI/4., se2.angle());
}

TEST(TEST_SE2, TEST_SE2_COEFFS)
{
  SE2d se2(4, 2, 0);

  EXPECT_DOUBLE_EQ(4, se2.coeffs()(0));
  EXPECT_DOUBLE_EQ(2, se2.coeffs()(1));
  EXPECT_DOUBLE_EQ(1, se2.coeffs()(2));
  EXPECT_DOUBLE_EQ(0, se2.coeffs()(3));
}

TEST(TEST_SE2, TEST_SE2_DATA)
{
  SE2d se2(4, 2, 0);

  double * data_ptr = se2.data();

  ASSERT_NE(nullptr, data_ptr);

  EXPECT_DOUBLE_EQ(4, *data_ptr);
  ++data_ptr;
  EXPECT_DOUBLE_EQ(2, *data_ptr);
  ++data_ptr;
  EXPECT_DOUBLE_EQ(1, *data_ptr);
  ++data_ptr;
  EXPECT_DOUBLE_EQ(0, *data_ptr);
}

TEST(TEST_SE2, TEST_SE2_CAST)
{
  SE2d se2d(4, 2, 0.1789);

  EXPECT_DOUBLE_EQ(4, se2d.x());
  EXPECT_DOUBLE_EQ(2, se2d.y());
  EXPECT_DOUBLE_EQ(0.1789, se2d.angle());

  SE2f se2f = se2d.cast<float>();

  EXPECT_FLOAT_EQ(4, se2f.x());
  EXPECT_FLOAT_EQ(2, se2f.y());
  EXPECT_FLOAT_EQ(0.1789, se2f.angle());
}

TEST(TEST_SE2, TEST_SE2_IDENTITY)
{
  SE2d se2;

  se2.setIdentity();

  EXPECT_DOUBLE_EQ(0, se2.x());
  EXPECT_DOUBLE_EQ(0, se2.y());
  EXPECT_DOUBLE_EQ(0, se2.angle());
}

TEST(TEST_SE2, TEST_SE2_IDENTITY2)
{
  SE2d se2 = SE2d::Identity();

  EXPECT_DOUBLE_EQ(0, se2.x());
  EXPECT_DOUBLE_EQ(0, se2.y());
  EXPECT_DOUBLE_EQ(0, se2.angle());
}

TEST(TEST_SE2, TEST_SE2_RANDOM)
{
  SE2d se2;

  se2.setRandom();

  const auto complex = se2.coeffs().block<2,1>(2,0);

  EXPECT_DOUBLE_EQ(1, complex.norm());
}

TEST(TEST_SE2, TEST_SE2_RANDOM2)
{
  const SE2d se2 = SE2d::Random();

  const auto complex = se2.coeffs().block<2,1>(2,0);

  EXPECT_DOUBLE_EQ(1, complex.norm());
}

TEST(TEST_SE2, TEST_SE2_MATRIX)
{
  SE2d se2 = SE2d::Identity();

  SE2d::Transformation t = se2.transform();

  EXPECT_EQ(3, t.rows());
  EXPECT_EQ(3, t.cols());

  /// @todo Eigen matrix comparison
}

TEST(TEST_SE2, TEST_SE2_ROTATION)
{
  SE2d se2 = SE2d::Identity();

  SE2d::Rotation r = se2.rotation();

  EXPECT_EQ(2, r.rows());
  EXPECT_EQ(2, r.cols());

  /// @todo Eigen matrix comparison
}

TEST(TEST_SE2, TEST_SE2_ASSIGN_OP)
{
  SE2d se2a(0, 0, 0);
  SE2d se2b(4, 2, MANIF_PI);

  se2a = se2b;

  EXPECT_DOUBLE_EQ(4, se2a.x());
  EXPECT_DOUBLE_EQ(2, se2a.y());
  EXPECT_DOUBLE_EQ(MANIF_PI, se2a.angle());
}

TEST(TEST_SE2, TEST_SE2_INVERSE)
{
  SE2d se2 = SE2d::Identity();

  auto se2_inv = se2.inverse();

  EXPECT_DOUBLE_EQ(0, se2_inv.x());
  EXPECT_DOUBLE_EQ(0, se2_inv.y());
  EXPECT_DOUBLE_EQ(0, se2_inv.angle());
  EXPECT_DOUBLE_EQ(1, se2_inv.real());
  EXPECT_DOUBLE_EQ(0, se2_inv.imag());

  se2 = SE2d(1, 1, MANIF_PI);
  se2_inv = se2.inverse();

  EXPECT_DOUBLE_EQ( 1, se2_inv.x());
  EXPECT_DOUBLE_EQ( 1, se2_inv.y());
  EXPECT_DOUBLE_EQ(-MANIF_PI, se2_inv.angle());
  EXPECT_DOUBLE_EQ(-1, se2_inv.real());
//  EXPECT_DOUBLE_EQ(0, se2_inv.imag());
  EXPECT_NEAR(0, se2_inv.imag(), 1e-15);


  se2 = SE2d(0.7, 2.3, MANIF_PI/3.);
  se2_inv = se2.inverse();

  EXPECT_DOUBLE_EQ(-2.341858428704209, se2_inv.x());
  EXPECT_DOUBLE_EQ(-0.543782217350893, se2_inv.y());
//  EXPECT_DOUBLE_EQ(-1.04719755119660,  se2_inv.angle());
  EXPECT_NEAR(-1.04719755119660, se2_inv.angle(), 3e-15);
}

TEST(TEST_SE2, TEST_SE2_RPLUS_ZERO)
{
  SE2d se2a(1, 1, MANIF_PI / 2.);
  SE2Tangentd se2b(0, 0, 0);

  auto se2c = se2a.rplus(se2b);

  EXPECT_DOUBLE_EQ(1, se2c.x());
  EXPECT_DOUBLE_EQ(1, se2c.y());
  EXPECT_DOUBLE_EQ(MANIF_PI/2., se2c.angle());
}

TEST(TEST_SE2, TEST_SE2_RPLUS)
{
  SE2d se2a(1, 1, MANIF_PI / 2.);
  SE2Tangentd se2b(1, 1, MANIF_PI / 2.);

  auto se2c = se2a.rplus(se2b);

  /// @todo what to expect here ?? :S
//  EXPECT_DOUBLE_EQ(0, se2c.x());
//  EXPECT_DOUBLE_EQ(2, se2c.y());
  EXPECT_DOUBLE_EQ(MANIF_PI, se2c.angle());
}

TEST(TEST_SE2, TEST_SE2_LPLUS_ZERO)
{
  SE2d se2a(1, 1, MANIF_PI / 2.);
  SE2Tangentd se2b(0, 0, 0);

  auto se2c = se2a.lplus(se2b);

  EXPECT_DOUBLE_EQ(1, se2c.x());
  EXPECT_DOUBLE_EQ(1, se2c.y());
  EXPECT_DOUBLE_EQ(MANIF_PI / 2., se2c.angle());
}

TEST(TEST_SE2, TEST_SE2_LPLUS)
{
  SE2d se2a(1, 1, MANIF_PI / 2.);
  SE2Tangentd se2b(1, 1, MANIF_PI / 2.);

  auto se2c = se2a.lplus(se2b);

  /// @todo what to expect here ?? :S
//  EXPECT_DOUBLE_EQ(1, se2c.x());
//  EXPECT_DOUBLE_EQ(1, se2c.y());
  EXPECT_DOUBLE_EQ(MANIF_PI, se2c.angle());
}

TEST(TEST_SE2, TEST_SE2_PLUS)
{
  SE2d se2a(1, 1, MANIF_PI / 2.);
  SE2Tangentd se2b(1, 1, MANIF_PI / 2.);

  auto se2c = se2a.plus(se2b);

  /// @todo what to expect here ?? :S
//  EXPECT_DOUBLE_EQ(0, se2c.x());
//  EXPECT_DOUBLE_EQ(2, se2c.y());
  EXPECT_DOUBLE_EQ(MANIF_PI, se2c.angle());
}

TEST(TEST_SE2, TEST_SE2_OP_PLUS)
{
  SE2d se2a(1, 1, MANIF_PI / 2.);
  SE2Tangentd se2b(1, 1, MANIF_PI / 2.);

  auto se2c = se2a + se2b;

  /// @todo what to expect here ?? :S
//  EXPECT_DOUBLE_EQ(0, se2c.x());
//  EXPECT_DOUBLE_EQ(2, se2c.y());
  EXPECT_DOUBLE_EQ(MANIF_PI, se2c.angle());
}

TEST(TEST_SE2, TEST_SE2_OP_PLUS_EQ)
{
  SE2d se2a(1, 1, MANIF_PI / 2.);
  SE2Tangentd se2b(1, 1, MANIF_PI / 2.);

  se2a += se2b;

  /// @todo what to expect here ?? :S
//  EXPECT_DOUBLE_EQ(0, se2a.x());
//  EXPECT_DOUBLE_EQ(2, se2a.y());
  EXPECT_DOUBLE_EQ(MANIF_PI, se2a.angle());
}

TEST(TEST_SE2, TEST_SE2_RMINUS_ZERO)
{
  SE2d se2a(0, 0, 0);
  SE2d se2b(0, 0, 0);

  auto se2c = se2a.rminus(se2b);

//  EXPECT_DOUBLE_EQ(0, se2c.x());
  EXPECT_NEAR(0, se2c.x(), 1e-15);
//  EXPECT_DOUBLE_EQ(0, se2c.y());
  EXPECT_NEAR(0, se2c.y(), 1e-15);
  EXPECT_DOUBLE_EQ(0, se2c.angle());
}

TEST(TEST_SE2, TEST_SE2_RMINUS_I)
{
  SE2d se2a(1, 1, MANIF_PI);
  SE2d se2b(1, 1, MANIF_PI);

  auto se2c = se2a.rminus(se2b);

  /// @todo
//  EXPECT_DOUBLE_EQ(0, se2c.x());
//  EXPECT_NEAR(0, se2c.x(), 1e-15);
//  EXPECT_DOUBLE_EQ(0, se2c.y());
//  EXPECT_NEAR(0, se2c.y(), 1e-15);
  EXPECT_DOUBLE_EQ(0, se2c.angle());
}

TEST(TEST_SE2, TEST_SE2_RMINUS)
{
  SE2d se2a(1, 1, MANIF_PI);
  SE2d se2b(2, 2, MANIF_PI_2);

  auto se2c = se2a.rminus(se2b);

  /// @todo
//  EXPECT_DOUBLE_EQ(1, se2c.x());
//  EXPECT_DOUBLE_EQ(1, se2c.y());
  EXPECT_DOUBLE_EQ(MANIF_PI_2, se2c.angle());
}

TEST(TEST_SE2, TEST_SE2_LMINUS_IDENTITY)
{
  SE2d se2a(0,0,0);
  SE2d se2b(0,0,0);

  auto se2c = se2a.lminus(se2b);

  /// @todo
  EXPECT_DOUBLE_EQ(0, se2c.x());
  EXPECT_DOUBLE_EQ(0, se2c.y());
  EXPECT_DOUBLE_EQ(0, se2c.angle());
}

TEST(TEST_SE2, TEST_SE2_LMINUS)
{
  SE2d se2a(1,1,MANIF_PI);
  SE2d se2b(2,2,MANIF_PI_2);

  auto se2c = se2a.lminus(se2b);

  /// @todo
//  EXPECT_DOUBLE_EQ(0, se2c.x());
//  EXPECT_DOUBLE_EQ(0, se2c.y());
  EXPECT_DOUBLE_EQ(MANIF_PI_2, se2c.angle());
}

TEST(TEST_SE2, TEST_SE2_MINUS)
{
  SE2d se2a(1, 1, MANIF_PI);
  SE2d se2b(2, 2, MANIF_PI_2);

  auto se2c = se2a.minus(se2b);

  /// @todo
//  EXPECT_DOUBLE_EQ(1, se2c.x());
//  EXPECT_DOUBLE_EQ(1, se2c.y());
  EXPECT_DOUBLE_EQ(MANIF_PI_2, se2c.angle());
}

TEST(TEST_SE2, TEST_SE2_LIFT)
{
  SE2d se2(1,1,MANIF_PI);

  auto se2_log = se2.log();

  /// @todo
//  EXPECT_DOUBLE_EQ(1, se2_log.x());
//  EXPECT_DOUBLE_EQ(1, se2_log.y());
  EXPECT_DOUBLE_EQ(MANIF_PI, se2_log.angle());
}

TEST(TEST_SE2, TEST_SE2_COMPOSE)
{
  SE2d se2a(1,1,MANIF_PI_2);
  SE2d se2b(2,2,MANIF_PI_2);

  auto se2c = se2a.compose(se2b);

  EXPECT_DOUBLE_EQ(-1, se2c.x());
  EXPECT_DOUBLE_EQ(+3, se2c.y());
  EXPECT_DOUBLE_EQ(MANIF_PI, se2c.angle());
}

TEST(TEST_SE2, TEST_SE2_OP_COMPOSE)
{
  SE2d se2a(1,1,MANIF_PI_2);
  SE2d se2b(2,2,MANIF_PI_2);

  auto se2c = se2a * se2b;

  EXPECT_DOUBLE_EQ(-1, se2c.x());
  EXPECT_DOUBLE_EQ(+3, se2c.y());
  EXPECT_DOUBLE_EQ(MANIF_PI, se2c.angle());
}

TEST(TEST_SE2, TEST_SE2_OP_COMPOSE_EQ)
{
  SE2d se2a(1,1,MANIF_PI_2);
  SE2d se2b(2,2,MANIF_PI_2);

  se2a *= se2b;

  EXPECT_DOUBLE_EQ(-1, se2a.x());
  EXPECT_DOUBLE_EQ(+3, se2a.y());
  EXPECT_DOUBLE_EQ(MANIF_PI, se2a.angle());
}

TEST(TEST_SE2, TEST_SE2_BETWEEN_I)
{
  SE2d se2a(1,1,MANIF_PI);
  SE2d se2b(1,1,MANIF_PI);

  auto se2c = se2a.between(se2b);

  EXPECT_DOUBLE_EQ(0, se2c.x());
  EXPECT_DOUBLE_EQ(0, se2c.y());
  EXPECT_DOUBLE_EQ(0, se2c.angle());
}

TEST(TEST_SE2, TEST_SE2_BETWEEN)
{
  SE2d se2a(1,1,MANIF_PI);
  SE2d se2b(2,2,MANIF_PI_2);

  auto se2c = se2a.between(se2b);

  EXPECT_DOUBLE_EQ(-1, se2c.x());
  EXPECT_DOUBLE_EQ(-1, se2c.y());
  EXPECT_DOUBLE_EQ(-MANIF_PI_2, se2c.angle());
}

TEST(TEST_SE2, TEST_SE2_ACT)
{
  SE2d se2(1,1,MANIF_PI/2.);

  auto transformed_point = se2.act(Eigen::Vector2d(1,1));

  /// @todo precision issue ?
  //EXPECT_DOUBLE_EQ(0.0, transformed_point.x());
  //EXPECT_DOUBLE_EQ(0.0, transformed_point.y());

  EXPECT_NEAR(0, transformed_point.x(), 1e-15);
  EXPECT_NEAR(2, transformed_point.y(), 1e-15);

  se2 = SE2d(1,1,-MANIF_PI/2.);

  transformed_point = se2.act(Eigen::Vector2d(1,1));

  EXPECT_NEAR(2, transformed_point.x(), 1e-15);
  EXPECT_NEAR(0, transformed_point.y(), 1e-15);

  se2 = SE2d::Identity();

  transformed_point = se2.act(Eigen::Vector2d(1,1));

  EXPECT_NEAR(1, transformed_point.x(), 1e-15);
  EXPECT_NEAR(1, transformed_point.y(), 1e-15);
}

MANIF_TEST(SE2d);

MANIF_TEST_JACOBIANS(SE2d);

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
