#include <gtest/gtest.h>

#include "../test_utils.h"

#include "manif/SO2.h"

using namespace manif;

TEST(TEST_SO2, TEST_SO2_BASE_TEMPLATE_DATA)
{
  SO2d so2(0);

  const auto& data = callCoeffs(so2);

  EXPECT_DOUBLE_EQ(1, data(0));
  EXPECT_DOUBLE_EQ(0, data(1));
}

TEST(TEST_SO2, TEST_SO2_BASE_TEMPLATE_IDENTITY)
{
  SO2d so2;

  callIdentity(so2);

  EXPECT_DOUBLE_EQ(0, so2.angle());
  EXPECT_DOUBLE_EQ(1, so2.real());
  EXPECT_DOUBLE_EQ(0, so2.imag());
}

/*
TEST(TEST_SO2, TEST_SO2_BASE_TEMPLATE_IDENTITY2)
{
  SO2d so2 = SO2d::Identity();

  EXPECT_DOUBLE_EQ(0, so2.angle());
  EXPECT_DOUBLE_EQ(1, so2.real());
  EXPECT_DOUBLE_EQ(0, so2.imag());
}
*/

TEST(TEST_SO2, TEST_SO2_BASE_TEMPLATE_RANDOM)
{
  SO2d so2;

  callRandom(so2);

  const SO2d& so2_ref = so2;

  EXPECT_DOUBLE_EQ(1, so2_ref.coeffs().norm());
}

//TEST(TEST_SO2, TEST_SO2_BASE_TEMPLATE_RANDOM2)
//{
//  SO2d so2 = SO2d::Random();

//  EXPECT_DOUBLE_EQ(0, so2.angle());
//}

TEST(TEST_SO2, TEST_SO2_BASE_TEMPLATE_MATRIX)
{
  SO2d::Transformation t = callTransform(SO2d::Identity());

  EXPECT_EQ(3, t.rows());
  EXPECT_EQ(3, t.cols());

  /// @todo Eigen matrix comparison
}

TEST(TEST_SO2, TEST_SO2_BASE_TEMPLATE_ROTATION)
{
  SO2d::Rotation r = callRotation(SO2d::Identity());

  EXPECT_EQ(2, r.rows());
  EXPECT_EQ(2, r.cols());

  /// @todo Eigen matrix comparison
}

TEST(TEST_SO2, TEST_SO2_BASE_TEMPLATE_INVERSE)
{
  SO2d so2 = SO2d::Identity();

  auto so2_inv = callInverse(so2);

  EXPECT_DOUBLE_EQ(so2.angle(), so2_inv.angle());
  EXPECT_DOUBLE_EQ(1, so2_inv.real());
  EXPECT_DOUBLE_EQ(0, so2_inv.imag());

  so2 = SO2d(M_PI);
  so2_inv = callInverse(so2);

  EXPECT_DOUBLE_EQ(-M_PI, so2_inv.angle());
}

TEST(TEST_SO2, TEST_SO2_BASE_TEMPLATE_RPLUS)
{
  SO2d so2a(M_PI / 2.);
  SO2Tangentd so2b(M_PI / 2.);

  auto so2c = callRplus(so2a, so2b);

  EXPECT_DOUBLE_EQ(M_PI, so2c.angle());
}

TEST(TEST_SO2, TEST_SO2_BASE_TEMPLATE_LPLUS)
{
  SO2d so2a(M_PI / 2.);
  SO2Tangentd so2b(M_PI / 2.);

  auto so2c = callLplus(so2a, so2b);

  EXPECT_DOUBLE_EQ(M_PI, so2c.angle());
}

TEST(TEST_SO2, TEST_SO2_BASE_TEMPLATE_PLUS)
{
  SO2d so2a(M_PI / 2.);
  SO2Tangentd so2b(M_PI / 2.);

  auto so2c = callPlus(so2a, so2b);

  EXPECT_DOUBLE_EQ(M_PI, so2c.angle());
}

TEST(TEST_SO2, TEST_SO2_BASE_TEMPLATE_OP_PLUS)
{
  SO2d so2a(M_PI / 2.);
  SO2Tangentd so2b(M_PI / 2.);

  auto so2c = callOpPlus(so2a, so2b);

  EXPECT_DOUBLE_EQ(M_PI, so2c.angle());
}

TEST(TEST_SO2, TEST_SO2_BASE_TEMPLATE_OP_PLUS_EQ)
{
  SO2d so2a(M_PI / 2.);
  SO2Tangentd so2b(M_PI / 2.);

  callOpPlusEq(so2a, so2b);

  EXPECT_DOUBLE_EQ(M_PI, so2a.angle());
}

TEST(TEST_SO2, TEST_SO2_BASE_TEMPLATE_RMINUS)
{
  SO2d so2a(M_PI);
  SO2d so2b(M_PI_2);

  auto so2c = callRminus(so2a, so2b);

  EXPECT_DOUBLE_EQ(M_PI_2, so2c.angle());
}

TEST(TEST_SO2, TEST_SO2_BASE_TEMPLATE_LMINUS)
{
  SO2d so2a(M_PI);
  SO2d so2b(M_PI_2);

  auto so2c = callLminus(so2a, so2b);

  EXPECT_DOUBLE_EQ(-M_PI_2, so2c.angle());
}

TEST(TEST_SO2, TEST_SO2_BASE_TEMPLATE_MINUS)
{
  SO2d so2a(M_PI);
  SO2d so2b(M_PI_2);

  auto so2c = callMinus(so2a, so2b);

  EXPECT_DOUBLE_EQ(M_PI_2, so2c.angle());
}

TEST(TEST_SO2, TEST_SO2_BASE_TEMPLATE_LIFT)
{
  SO2d so2(M_PI);

  auto so2_lift = callLift(so2);

  EXPECT_DOUBLE_EQ(M_PI, so2_lift.angle());
}

TEST(TEST_SO2, TEST_SO2_BASE_TEMPLATE_COMPOSE)
{
  SO2d so2a(M_PI_2);
  SO2d so2b(M_PI_2);

  auto so2c = callCompose(so2a, so2b);

  EXPECT_DOUBLE_EQ(M_PI, so2c.angle());
}

TEST(TEST_SO2, TEST_SO2_BASE_TEMPLATE_OP_COMPOSE)
{
  SO2d so2a(M_PI_2);
  SO2d so2b(M_PI_2);

  auto so2c = callOpTime(so2a, so2b);

  EXPECT_DOUBLE_EQ(M_PI, so2c.angle());
}

TEST(TEST_SO2, TEST_SO2_BASE_TEMPLATE_OP_COMPOSE_EQ)
{
  SO2d so2a(M_PI_2);
  SO2d so2b(M_PI_2);

  callOpTimeEq(so2a, so2b);

  EXPECT_DOUBLE_EQ(M_PI, so2a.angle());
}

TEST(TEST_SO2, TEST_SO2_BASE_TEMPLATE_BETWEEN)
{
  SO2d so2a(M_PI);
  SO2d so2b(M_PI_2);

  auto so2c = callBetween(so2a, so2b);

  EXPECT_DOUBLE_EQ(-M_PI_2, so2c.angle());
}

/// with Jacs
/*
TEST(TEST_SO2, TEST_SO2_BASE_TEMPLATE_INVERSE_JAC)
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

  so2.angle(M_PI);
  so2.inverse(so2_inv, J_inv);

  EXPECT_DOUBLE_EQ(-M_PI, so2_inv.angle());

  EXPECT_EQ(1, J_inv.rows());
  EXPECT_EQ(1, J_inv.cols());
  EXPECT_DOUBLE_EQ(-1, J_inv(0));
}

TEST(TEST_SO2, TEST_SO2_BASE_TEMPLATE_LIFT_JAC)
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

/// @todo move to SO2Tangent tests
TEST(TEST_SO2, TEST_SO2_BASE_TEMPLATE_RETRACT_JAC)
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

TEST(TEST_SO2, TEST_SO2_BASE_TEMPLATE_COMPOSE_JAC)
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

TEST(TEST_SO2, TEST_SO2_BASE_TEMPLATE_RPLUS_JAC)
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

TEST(TEST_SO2, TEST_SO2_BASE_TEMPLATE_LPLUS_JAC)
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

TEST(TEST_SO2, TEST_SO2_BASE_TEMPLATE_PLUS_JAC)
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

TEST(TEST_SO2, TEST_SO2_BASE_TEMPLATE_RMINUS_JAC)
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

TEST(TEST_SO2, TEST_SO2_BASE_TEMPLATE_LMINUS_JAC)
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

TEST(TEST_SO2, TEST_SO2_BASE_TEMPLATE_MINUS_JAC)
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
*/
int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
