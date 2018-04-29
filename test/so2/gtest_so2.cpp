#include <gtest/gtest.h>

#include "manif/SO2.h"

using namespace manif;

TEST(TEST_SO2, TEST_SO2_0)
{
  SO2d so2(SO2d::DataType(1,0));

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
  SO2d so2(0);

  EXPECT_DOUBLE_EQ(1, so2.coeffs()(0));
  EXPECT_DOUBLE_EQ(0, so2.coeffs()(1));
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

TEST(TEST_SO2, TEST_SO2_RANDOM)
{
  SO2d so2;

  so2.random();

  EXPECT_DOUBLE_EQ(1, so2.coeffs().norm());
}

TEST(TEST_SO2, TEST_SO2_RANDOM2)
{
  const SO2d so2 = SO2d::Random();

  EXPECT_DOUBLE_EQ(1, so2.coeffs().norm());
}

TEST(TEST_SO2, TEST_SO2_MATRIX)
{
  SO2d so2 = SO2d::Identity();

  SO2d::Transformation t = so2.transform();

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

TEST(TEST_SO2, TEST_SO2_ASSIGN_OP)
{
  SO2d so2a(0);
  SO2d so2b(M_PI);

  so2a = so2b;

  EXPECT_DOUBLE_EQ(M_PI, so2a.angle());
}

TEST(TEST_SO2, TEST_SO2_INVERSE)
{
  SO2d so2 = SO2d::Identity();

  auto so2_inv = so2.inverse();

  EXPECT_DOUBLE_EQ(so2.angle(), so2_inv.angle());
  EXPECT_DOUBLE_EQ(1, so2_inv.real());
  EXPECT_DOUBLE_EQ(0, so2_inv.imag());

  so2 = SO2d(M_PI);
  so2_inv = so2.inverse();

  EXPECT_DOUBLE_EQ(-M_PI, so2_inv.angle());
}

TEST(TEST_SO2, TEST_SO2_RPLUS)
{
  SO2d so2a(M_PI / 2.);
  SO2Tangentd so2b(M_PI / 2.);

  auto so2c = so2a.rplus(so2b);

  EXPECT_DOUBLE_EQ(M_PI, so2c.angle());
}

TEST(TEST_SO2, TEST_SO2_LPLUS)
{
  SO2d so2a(M_PI / 2.);
  SO2Tangentd so2b(M_PI / 2.);

  auto so2c = so2a.lplus(so2b);

  EXPECT_DOUBLE_EQ(M_PI, so2c.angle());
}

TEST(TEST_SO2, TEST_SO2_PLUS)
{
  SO2d so2a(M_PI / 2.);
  SO2Tangentd so2b(M_PI / 2.);

  auto so2c = so2a.plus(so2b);

  EXPECT_DOUBLE_EQ(M_PI, so2c.angle());
}

TEST(TEST_SO2, TEST_SO2_OP_PLUS)
{
  SO2d so2a(M_PI / 2.);
  SO2Tangentd so2b(M_PI / 2.);

  auto so2c = so2a + so2b;

  EXPECT_DOUBLE_EQ(M_PI, so2c.angle());
}

TEST(TEST_SO2, TEST_SO2_OP_PLUS_EQ)
{
  SO2d so2a(M_PI / 2.);
  SO2Tangentd so2b(M_PI / 2.);

  so2a += so2b;

  EXPECT_DOUBLE_EQ(M_PI, so2a.angle());
}

TEST(TEST_SO2, TEST_SO2_RMINUS)
{
  SO2d so2a(M_PI);
  SO2d so2b(M_PI_2);

  auto so2c = so2a.rminus(so2b);

  EXPECT_DOUBLE_EQ(M_PI_2, so2c.angle());
}

TEST(TEST_SO2, TEST_SO2_LMINUS)
{
  SO2d so2a(M_PI);
  SO2d so2b(M_PI_2);

  auto so2c = so2a.lminus(so2b);

  EXPECT_DOUBLE_EQ(-M_PI_2, so2c.angle());
}

TEST(TEST_SO2, TEST_SO2_MINUS)
{
  SO2d so2a(M_PI);
  SO2d so2b(M_PI_2);

  auto so2c = so2a.minus(so2b);

  EXPECT_DOUBLE_EQ(M_PI_2, so2c.angle());
}

TEST(TEST_SO2, TEST_SO2_LIFT)
{
  SO2d so2(M_PI);

  auto so2_lift = so2.lift();

  EXPECT_DOUBLE_EQ(M_PI, so2_lift.angle());
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

/// with Jacs

TEST(TEST_SO2, TEST_SO2_INVERSE_JAC)
{
  SO2d so2 = SO2d::Identity();

  SO2d so2_inv;
  SO2d::Jacobian J_inv;
  so2.inverse(so2_inv, J_inv);

  EXPECT_DOUBLE_EQ(so2.angle(), so2_inv.angle());
  EXPECT_DOUBLE_EQ(1, so2_inv.real());
  EXPECT_DOUBLE_EQ(0, so2_inv.imag());

  EXPECT_EQ(1, J_inv.rows());
  EXPECT_EQ(1, J_inv.cols());
  EXPECT_DOUBLE_EQ(-1, J_inv(0));

  so2 = SO2d(M_PI);
  so2.inverse(so2_inv, J_inv);

  EXPECT_DOUBLE_EQ(-M_PI, so2_inv.angle());

  EXPECT_EQ(1, J_inv.rows());
  EXPECT_EQ(1, J_inv.cols());
  EXPECT_DOUBLE_EQ(-1, J_inv(0));
}

TEST(TEST_SO2, TEST_SO2_LIFT_JAC)
{
  SO2d so2(M_PI);

  SO2d::Tangent so2_lift;
  SO2d::Tangent::Jacobian J_lift;

  so2.lift(so2_lift, J_lift);

  EXPECT_DOUBLE_EQ(M_PI, so2_lift.angle());

  /// @todo check this J
  EXPECT_EQ(1, J_lift.rows());
  EXPECT_EQ(1, J_lift.cols());
  EXPECT_DOUBLE_EQ(1, J_lift(0));
}

TEST(TEST_SO2, TEST_SO2_COMPOSE_JAC)
{
  SO2d so2a(M_PI_2);
  SO2d so2b(M_PI_2);

  SO2d so2c;
  SO2d::Jacobian J_c_a, J_c_b;

  so2a.compose(so2b, so2c, J_c_a, J_c_b);

  EXPECT_DOUBLE_EQ(M_PI, so2c.angle());

  EXPECT_EQ(1, J_c_a.rows());
  EXPECT_EQ(1, J_c_a.cols());
  EXPECT_DOUBLE_EQ(1, J_c_a(0));

  EXPECT_EQ(1, J_c_b.rows());
  EXPECT_EQ(1, J_c_b.cols());
  EXPECT_DOUBLE_EQ(1, J_c_b(0));
}

TEST(TEST_SO2, TEST_SO2_RPLUS_JAC)
{
  SO2d so2a(M_PI / 2.);
  SO2Tangentd so2b(M_PI / 2.);

  SO2d so2c;
  SO2d::Jacobian J_rplus_m;
  SO2d::Jacobian J_rplus_t;

  so2a.rplus(so2b, so2c, J_rplus_m, J_rplus_t);

  EXPECT_DOUBLE_EQ(M_PI, so2c.angle());

  EXPECT_EQ(1, J_rplus_m.rows());
  EXPECT_EQ(1, J_rplus_m.cols());
  EXPECT_DOUBLE_EQ(1, J_rplus_m(0));

  EXPECT_EQ(1, J_rplus_t.rows());
  EXPECT_EQ(1, J_rplus_t.cols());
  EXPECT_DOUBLE_EQ(1, J_rplus_t(0));
}

TEST(TEST_SO2, TEST_SO2_LPLUS_JAC)
{
  SO2d so2a(M_PI / 2.);
  SO2Tangentd so2b(M_PI / 2.);

  SO2d so2c;
  SO2d::Jacobian J_lplus_t;
  SO2d::Jacobian J_lplus_m;

  so2a.lplus(so2b, so2c, J_lplus_t, J_lplus_m);

  EXPECT_DOUBLE_EQ(M_PI, so2c.angle());

  EXPECT_EQ(1, J_lplus_t.rows());
  EXPECT_EQ(1, J_lplus_t.cols());
  EXPECT_DOUBLE_EQ(1, J_lplus_t(0));

  EXPECT_EQ(1, J_lplus_m.rows());
  EXPECT_EQ(1, J_lplus_m.cols());
  EXPECT_DOUBLE_EQ(1, J_lplus_m(0));
}

TEST(TEST_SO2, TEST_SO2_PLUS_JAC)
{
  SO2d so2a(M_PI / 2.);
  SO2Tangentd so2b(M_PI / 2.);

  SO2d so2c;
  SO2d::Jacobian J_plus_m;
  SO2d::Jacobian J_plus_t;

  so2a.plus(so2b, so2c, J_plus_m, J_plus_t);

  EXPECT_DOUBLE_EQ(M_PI, so2c.angle());

  EXPECT_EQ(1, J_plus_m.rows());
  EXPECT_EQ(1, J_plus_m.cols());
  EXPECT_DOUBLE_EQ(1, J_plus_m(0));

  EXPECT_EQ(1, J_plus_t.rows());
  EXPECT_EQ(1, J_plus_t.cols());
  EXPECT_DOUBLE_EQ(1, J_plus_t(0));
}

TEST(TEST_SO2, TEST_SO2_RMINUS_JAC)
{
  SO2d so2a(M_PI);
  SO2d so2b(M_PI_2);

  SO2Tangentd so2c;

  SO2d::Jacobian J_rminus_a, J_rminus_b;

  so2a.rminus(so2b, so2c, J_rminus_a, J_rminus_b);

  EXPECT_DOUBLE_EQ(M_PI_2, so2c.angle());

  EXPECT_EQ(1, J_rminus_a.rows());
  EXPECT_EQ(1, J_rminus_a.cols());
  EXPECT_DOUBLE_EQ(1, J_rminus_a(0));

  EXPECT_EQ(1, J_rminus_b.rows());
  EXPECT_EQ(1, J_rminus_b.cols());
  EXPECT_DOUBLE_EQ(-1, J_rminus_b(0));
}

TEST(TEST_SO2, TEST_SO2_LMINUS_JAC)
{
  SO2d so2a(M_PI);
  SO2d so2b(M_PI_2);

  SO2Tangentd so2c;

  SO2d::Jacobian J_lminus_a, J_lminus_b;

  so2a.lminus(so2b, so2c, J_lminus_a, J_lminus_b);

  EXPECT_DOUBLE_EQ(-M_PI_2, so2c.angle());

  EXPECT_EQ(1, J_lminus_a.rows());
  EXPECT_EQ(1, J_lminus_a.cols());
  EXPECT_DOUBLE_EQ(-1, J_lminus_a(0));

  EXPECT_EQ(1, J_lminus_b.rows());
  EXPECT_EQ(1, J_lminus_b.cols());
  EXPECT_DOUBLE_EQ(1, J_lminus_b(0));
}

TEST(TEST_SO2, TEST_SO2_MINUS_JAC)
{
  SO2d so2a(M_PI);
  SO2d so2b(M_PI_2);

  SO2Tangentd so2c;

  SO2d::Jacobian J_minus_a, J_minus_b;

  so2a.minus(so2b, so2c, J_minus_a, J_minus_b);

  EXPECT_DOUBLE_EQ(M_PI_2, so2c.angle());

  EXPECT_EQ(1, J_minus_a.rows());
  EXPECT_EQ(1, J_minus_a.cols());
  EXPECT_DOUBLE_EQ(1, J_minus_a(0));

  EXPECT_EQ(1, J_minus_b.rows());
  EXPECT_EQ(1, J_minus_b.cols());
  EXPECT_DOUBLE_EQ(-1, J_minus_b(0));
}

TEST(TEST_SO2, TEST_SO2_BETWEEN_JAC)
{
  SO2d so2a(M_PI);
  SO2d so2b(M_PI_2);

  SO2d::Jacobian J_between_a, J_between_b;
  SO2d so2c;

  so2a.between(so2b, so2c, J_between_a, J_between_b);

  EXPECT_DOUBLE_EQ(-M_PI_2, so2c.angle());

  EXPECT_EQ(1, J_between_a.rows());
  EXPECT_EQ(1, J_between_a.cols());
  EXPECT_DOUBLE_EQ(-1, J_between_a(0));

  EXPECT_EQ(1, J_between_b.rows());
  EXPECT_EQ(1, J_between_b.cols());
  EXPECT_DOUBLE_EQ(1, J_between_b(0));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
