#include <gtest/gtest.h>

#include "manif/SE2.h"

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
  EXPECT_DOUBLE_EQ(M_PI/4., se2.angle());
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

  se2.identity();

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

  se2.random();

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
  SE2d se2b(4, 2, M_PI);

  se2a = se2b;

  EXPECT_DOUBLE_EQ(4, se2a.x());
  EXPECT_DOUBLE_EQ(2, se2a.y());
  EXPECT_DOUBLE_EQ(M_PI, se2a.angle());
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

  se2 = SE2d(1, 1, M_PI);
  se2_inv = se2.inverse();

  EXPECT_DOUBLE_EQ( 1, se2_inv.x());
  EXPECT_DOUBLE_EQ( 1, se2_inv.y());
  EXPECT_DOUBLE_EQ(-M_PI, se2_inv.angle());
  EXPECT_DOUBLE_EQ(-1, se2_inv.real());
//  EXPECT_DOUBLE_EQ(0, se2_inv.imag());
  EXPECT_NEAR(0, se2_inv.imag(), 1e-15);


  se2 = SE2d(0.7, 2.3, M_PI/3.);
  se2_inv = se2.inverse();

  EXPECT_DOUBLE_EQ(-2.341858428704209, se2_inv.x());
  EXPECT_DOUBLE_EQ(-0.543782217350893, se2_inv.y());
//  EXPECT_DOUBLE_EQ(-1.04719755119660,  se2_inv.angle());
  EXPECT_NEAR(-1.04719755119660, se2_inv.angle(), 3e-15);
}

TEST(TEST_SE2, TEST_SE2_RPLUS_ZERO)
{
  SE2d se2a(1, 1, M_PI / 2.);
  SE2Tangentd se2b(0, 0, 0);

  auto se2c = se2a.rplus(se2b);

  EXPECT_DOUBLE_EQ(1, se2c.x());
  EXPECT_DOUBLE_EQ(1, se2c.y());
  EXPECT_DOUBLE_EQ(M_PI/2., se2c.angle());
}

TEST(TEST_SE2, TEST_SE2_RPLUS)
{
  SE2d se2a(1, 1, M_PI / 2.);
  SE2Tangentd se2b(1, 1, M_PI / 2.);

  auto se2c = se2a.rplus(se2b);

  /// @todo what to expect here ?? :S
//  EXPECT_DOUBLE_EQ(0, se2c.x());
//  EXPECT_DOUBLE_EQ(2, se2c.y());
  EXPECT_DOUBLE_EQ(M_PI, se2c.angle());
}

TEST(TEST_SE2, TEST_SE2_LPLUS_ZERO)
{
  SE2d se2a(1, 1, M_PI / 2.);
  SE2Tangentd se2b(0, 0, 0);

  auto se2c = se2a.lplus(se2b);

  EXPECT_DOUBLE_EQ(1, se2c.x());
  EXPECT_DOUBLE_EQ(1, se2c.y());
  EXPECT_DOUBLE_EQ(M_PI / 2., se2c.angle());
}

TEST(TEST_SE2, TEST_SE2_LPLUS)
{
  SE2d se2a(1, 1, M_PI / 2.);
  SE2Tangentd se2b(1, 1, M_PI / 2.);

  auto se2c = se2a.lplus(se2b);

  /// @todo what to expect here ?? :S
//  EXPECT_DOUBLE_EQ(1, se2c.x());
//  EXPECT_DOUBLE_EQ(1, se2c.y());
  EXPECT_DOUBLE_EQ(M_PI, se2c.angle());
}

TEST(TEST_SE2, TEST_SE2_PLUS)
{
  SE2d se2a(1, 1, M_PI / 2.);
  SE2Tangentd se2b(1, 1, M_PI / 2.);

  auto se2c = se2a.plus(se2b);

  /// @todo what to expect here ?? :S
//  EXPECT_DOUBLE_EQ(0, se2c.x());
//  EXPECT_DOUBLE_EQ(2, se2c.y());
  EXPECT_DOUBLE_EQ(M_PI, se2c.angle());
}

TEST(TEST_SE2, TEST_SE2_OP_PLUS)
{
  SE2d se2a(1, 1, M_PI / 2.);
  SE2Tangentd se2b(1, 1, M_PI / 2.);

  auto se2c = se2a + se2b;

  /// @todo what to expect here ?? :S
//  EXPECT_DOUBLE_EQ(0, se2c.x());
//  EXPECT_DOUBLE_EQ(2, se2c.y());
  EXPECT_DOUBLE_EQ(M_PI, se2c.angle());
}

TEST(TEST_SE2, TEST_SE2_OP_PLUS_EQ)
{
  SE2d se2a(1, 1, M_PI / 2.);
  SE2Tangentd se2b(1, 1, M_PI / 2.);

  se2a += se2b;

  /// @todo what to expect here ?? :S
//  EXPECT_DOUBLE_EQ(0, se2a.x());
//  EXPECT_DOUBLE_EQ(2, se2a.y());
  EXPECT_DOUBLE_EQ(M_PI, se2a.angle());
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
  SE2d se2a(1, 1, M_PI);
  SE2d se2b(1, 1, M_PI);

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
  SE2d se2a(1, 1, M_PI);
  SE2d se2b(2, 2, M_PI_2);

  auto se2c = se2a.rminus(se2b);

  /// @todo
//  EXPECT_DOUBLE_EQ(1, se2c.x());
//  EXPECT_DOUBLE_EQ(1, se2c.y());
  EXPECT_DOUBLE_EQ(M_PI_2, se2c.angle());
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
  SE2d se2a(1,1,M_PI);
  SE2d se2b(2,2,M_PI_2);

  auto se2c = se2a.lminus(se2b);

  /// @todo
//  EXPECT_DOUBLE_EQ(0, se2c.x());
//  EXPECT_DOUBLE_EQ(0, se2c.y());
  EXPECT_DOUBLE_EQ(-M_PI_2, se2c.angle());
}

TEST(TEST_SE2, TEST_SE2_MINUS)
{
  SE2d se2a(1, 1, M_PI);
  SE2d se2b(2, 2, M_PI_2);

  auto se2c = se2a.minus(se2b);

  /// @todo
//  EXPECT_DOUBLE_EQ(1, se2c.x());
//  EXPECT_DOUBLE_EQ(1, se2c.y());
  EXPECT_DOUBLE_EQ(M_PI_2, se2c.angle());
}

TEST(TEST_SE2, TEST_SE2_LIFT)
{
  SE2d se2(1,1,M_PI);

  auto se2_lift = se2.lift();

  /// @todo
//  EXPECT_DOUBLE_EQ(1, se2_lift.x());
//  EXPECT_DOUBLE_EQ(1, se2_lift.y());
  EXPECT_DOUBLE_EQ(M_PI, se2_lift.angle());
}

TEST(TEST_SE2, TEST_SE2_COMPOSE)
{
  SE2d se2a(1,1,M_PI_2);
  SE2d se2b(2,2,M_PI_2);

  auto se2c = se2a.compose(se2b);

  EXPECT_DOUBLE_EQ(-1, se2c.x());
  EXPECT_DOUBLE_EQ(+3, se2c.y());
  EXPECT_DOUBLE_EQ(M_PI, se2c.angle());
}

TEST(TEST_SE2, TEST_SE2_OP_COMPOSE)
{
  SE2d se2a(1,1,M_PI_2);
  SE2d se2b(2,2,M_PI_2);

  auto se2c = se2a * se2b;

  EXPECT_DOUBLE_EQ(-1, se2c.x());
  EXPECT_DOUBLE_EQ(+3, se2c.y());
  EXPECT_DOUBLE_EQ(M_PI, se2c.angle());
}

TEST(TEST_SE2, TEST_SE2_OP_COMPOSE_EQ)
{
  SE2d se2a(1,1,M_PI_2);
  SE2d se2b(2,2,M_PI_2);

  se2a *= se2b;

  EXPECT_DOUBLE_EQ(-1, se2a.x());
  EXPECT_DOUBLE_EQ(+3, se2a.y());
  EXPECT_DOUBLE_EQ(M_PI, se2a.angle());
}

TEST(TEST_SE2, TEST_SE2_BETWEEN_I)
{
  SE2d se2a(1,1,M_PI);
  SE2d se2b(1,1,M_PI);

  auto se2c = se2a.between(se2b);

  EXPECT_DOUBLE_EQ(0, se2c.x());
  EXPECT_DOUBLE_EQ(0, se2c.y());
  EXPECT_DOUBLE_EQ(0, se2c.angle());
}

TEST(TEST_SE2, TEST_SE2_BETWEEN)
{
  SE2d se2a(1,1,M_PI);
  SE2d se2b(2,2,M_PI_2);

  auto se2c = se2a.between(se2b);

  EXPECT_DOUBLE_EQ(-1, se2c.x());
  EXPECT_DOUBLE_EQ(-1, se2c.y());
  EXPECT_DOUBLE_EQ(-M_PI_2, se2c.angle());
}

/*
/// with Jacs

TEST(TEST_SE2, TEST_SE2_INVERSE_JAC)
{
  SE2d se2 = SE2d::Identity();

  SE2d se2_inv;
  SE2d::Jacobian J_inv;
  se2.inverse(se2_inv, J_inv);

  EXPECT_DOUBLE_EQ(se2.angle(), se2_inv.angle());
  EXPECT_DOUBLE_EQ(1, se2_inv.real());
  EXPECT_DOUBLE_EQ(0, se2_inv.imag());

  EXPECT_EQ(1, J_inv.rows());
  EXPECT_EQ(1, J_inv.cols());
  EXPECT_DOUBLE_EQ(-1, J_inv(0));

  se2 = SE2d(M_PI);
  se2.inverse(se2_inv, J_inv);

  EXPECT_DOUBLE_EQ(-M_PI, se2_inv.angle());

  EXPECT_EQ(1, J_inv.rows());
  EXPECT_EQ(1, J_inv.cols());
  EXPECT_DOUBLE_EQ(-1, J_inv(0));
}

TEST(TEST_SE2, TEST_SE2_LIFT_JAC)
{
  SE2d se2(M_PI);

  SE2d::Tangent se2_lift;
  SE2d::Tangent::Jacobian J_lift;

  se2.lift(se2_lift, J_lift);

  EXPECT_DOUBLE_EQ(M_PI, se2_lift.angle());

  /// @todo check this J
  EXPECT_EQ(1, J_lift.rows());
  EXPECT_EQ(1, J_lift.cols());
  EXPECT_DOUBLE_EQ(1, J_lift(0));
}

TEST(TEST_SE2, TEST_SE2_COMPOSE_JAC)
{
  SE2d se2a(M_PI_2);
  SE2d se2b(M_PI_2);

  SE2d se2c;
  SE2d::Jacobian J_c_a, J_c_b;

  se2a.compose(se2b, se2c, J_c_a, J_c_b);

  EXPECT_DOUBLE_EQ(M_PI, se2c.angle());

  EXPECT_EQ(1, J_c_a.rows());
  EXPECT_EQ(1, J_c_a.cols());
  EXPECT_DOUBLE_EQ(1, J_c_a(0));

  EXPECT_EQ(1, J_c_b.rows());
  EXPECT_EQ(1, J_c_b.cols());
  EXPECT_DOUBLE_EQ(1, J_c_b(0));
}

TEST(TEST_SE2, TEST_SE2_RPLUS_JAC)
{
  SE2d se2a(M_PI / 2.);
  SE2Tangentd se2b(M_PI / 2.);

  SE2d se2c;
  SE2d::Jacobian J_rplus_m;
  SE2d::Jacobian J_rplus_t;

  se2a.rplus(se2b, se2c, J_rplus_m, J_rplus_t);

  EXPECT_DOUBLE_EQ(M_PI, se2c.angle());

  EXPECT_EQ(1, J_rplus_m.rows());
  EXPECT_EQ(1, J_rplus_m.cols());
  EXPECT_DOUBLE_EQ(1, J_rplus_m(0));

  EXPECT_EQ(1, J_rplus_t.rows());
  EXPECT_EQ(1, J_rplus_t.cols());
  EXPECT_DOUBLE_EQ(1, J_rplus_t(0));
}

TEST(TEST_SE2, TEST_SE2_LPLUS_JAC)
{
  SE2d se2a(M_PI / 2.);
  SE2Tangentd se2b(M_PI / 2.);

  SE2d se2c;
  SE2d::Jacobian J_lplus_t;
  SE2d::Jacobian J_lplus_m;

  se2a.lplus(se2b, se2c, J_lplus_t, J_lplus_m);

  EXPECT_DOUBLE_EQ(M_PI, se2c.angle());

  EXPECT_EQ(1, J_lplus_t.rows());
  EXPECT_EQ(1, J_lplus_t.cols());
  EXPECT_DOUBLE_EQ(1, J_lplus_t(0));

  EXPECT_EQ(1, J_lplus_m.rows());
  EXPECT_EQ(1, J_lplus_m.cols());
  EXPECT_DOUBLE_EQ(1, J_lplus_m(0));
}

TEST(TEST_SE2, TEST_SE2_PLUS_JAC)
{
  SE2d se2a(M_PI / 2.);
  SE2Tangentd se2b(M_PI / 2.);

  SE2d se2c;
  SE2d::Jacobian J_plus_m;
  SE2d::Jacobian J_plus_t;

  se2a.plus(se2b, se2c, J_plus_m, J_plus_t);

  EXPECT_DOUBLE_EQ(M_PI, se2c.angle());

  EXPECT_EQ(1, J_plus_m.rows());
  EXPECT_EQ(1, J_plus_m.cols());
  EXPECT_DOUBLE_EQ(1, J_plus_m(0));

  EXPECT_EQ(1, J_plus_t.rows());
  EXPECT_EQ(1, J_plus_t.cols());
  EXPECT_DOUBLE_EQ(1, J_plus_t(0));
}

TEST(TEST_SE2, TEST_SE2_RMINUS_JAC)
{
  SE2d se2a(M_PI);
  SE2d se2b(M_PI_2);

  SE2Tangentd se2c;

  SE2d::Jacobian J_rminus_a, J_rminus_b;

  se2a.rminus(se2b, se2c, J_rminus_a, J_rminus_b);

  EXPECT_DOUBLE_EQ(M_PI_2, se2c.angle());

  EXPECT_EQ(1, J_rminus_a.rows());
  EXPECT_EQ(1, J_rminus_a.cols());
  EXPECT_DOUBLE_EQ(1, J_rminus_a(0));

  EXPECT_EQ(1, J_rminus_b.rows());
  EXPECT_EQ(1, J_rminus_b.cols());
  EXPECT_DOUBLE_EQ(-1, J_rminus_b(0));
}

TEST(TEST_SE2, TEST_SE2_LMINUS_JAC)
{
  SE2d se2a(M_PI);
  SE2d se2b(M_PI_2);

  SE2Tangentd se2c;

  SE2d::Jacobian J_lminus_a, J_lminus_b;

  se2a.lminus(se2b, se2c, J_lminus_a, J_lminus_b);

  EXPECT_DOUBLE_EQ(-M_PI_2, se2c.angle());

  EXPECT_EQ(1, J_lminus_a.rows());
  EXPECT_EQ(1, J_lminus_a.cols());
  EXPECT_DOUBLE_EQ(-1, J_lminus_a(0));

  EXPECT_EQ(1, J_lminus_b.rows());
  EXPECT_EQ(1, J_lminus_b.cols());
  EXPECT_DOUBLE_EQ(1, J_lminus_b(0));
}

TEST(TEST_SE2, TEST_SE2_MINUS_JAC)
{
  SE2d se2a(M_PI);
  SE2d se2b(M_PI_2);

  SE2Tangentd se2c;

  SE2d::Jacobian J_minus_a, J_minus_b;

  se2a.minus(se2b, se2c, J_minus_a, J_minus_b);

  EXPECT_DOUBLE_EQ(M_PI_2, se2c.angle());

  EXPECT_EQ(1, J_minus_a.rows());
  EXPECT_EQ(1, J_minus_a.cols());
  EXPECT_DOUBLE_EQ(1, J_minus_a(0));

  EXPECT_EQ(1, J_minus_b.rows());
  EXPECT_EQ(1, J_minus_b.cols());
  EXPECT_DOUBLE_EQ(-1, J_minus_b(0));
}

TEST(TEST_SE2, TEST_SE2_BETWEEN_JAC)
{
  SE2d se2a(M_PI);
  SE2d se2b(M_PI_2);

  SE2d::Jacobian J_between_a, J_between_b;
  SE2d se2c;

  se2a.between(se2b, se2c, J_between_a, J_between_b);

  EXPECT_DOUBLE_EQ(-M_PI_2, se2c.angle());

  EXPECT_EQ(1, J_between_a.rows());
  EXPECT_EQ(1, J_between_a.cols());
  EXPECT_DOUBLE_EQ(-1, J_between_a(0));

  EXPECT_EQ(1, J_between_b.rows());
  EXPECT_EQ(1, J_between_b.cols());
  EXPECT_DOUBLE_EQ(1, J_between_b(0));
}
*/

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

//  ::testing::GTEST_FLAG(filter) = "TEST_SE2.TEST_SE2_INVERSE";

  return RUN_ALL_TESTS();
}
