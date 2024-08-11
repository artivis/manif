#include <gtest/gtest.h>

#include "../common_tester.h"

#include "manif/SE2.h"

using namespace manif;

TEST(TEST_SE2, TEST_SE2_MAP_CONSTRUCTOR)
{
  double data[4] = {4,2,-1,0};
  Eigen::Map<SE2d> se2(data);

  EXPECT_DOUBLE_EQ(4,    se2.x());
  EXPECT_DOUBLE_EQ(2,    se2.y());
  EXPECT_DOUBLE_EQ(MANIF_PI, se2.angle());
}

TEST(TEST_SE2, TEST_SE2_MAP_COEFFS)
{
  double data[4] = {4,2,1,0};
  Eigen::Map<SE2d> se2(data);

  EXPECT_DOUBLE_EQ(4, se2.coeffs()(0));
  EXPECT_DOUBLE_EQ(2, se2.coeffs()(1));
  EXPECT_DOUBLE_EQ(1, se2.coeffs()(2));
  EXPECT_DOUBLE_EQ(0, se2.coeffs()(3));
}

TEST(TEST_SE2, TEST_SE2_MAP_DATA)
{
  double data[4] = {4,2,1,0};
  Eigen::Map<SE2d> se2(data);

  double * data_ptr = se2.data();

  ASSERT_NE(nullptr, data_ptr);
  EXPECT_EQ(data, data_ptr);

  EXPECT_DOUBLE_EQ(4, *data_ptr);
  ++data_ptr;
  EXPECT_DOUBLE_EQ(2, *data_ptr);
  ++data_ptr;
  EXPECT_DOUBLE_EQ(1, *data_ptr);
  ++data_ptr;
  EXPECT_DOUBLE_EQ(0, *data_ptr);
}

TEST(TEST_SE2, TEST_SE2_MAP_CAST)
{
  double data[4] = {4,2,-1,0};
  Eigen::Map<SE2d> se2d(data);

  EXPECT_DOUBLE_EQ(4, se2d.x());
  EXPECT_DOUBLE_EQ(2, se2d.y());
  EXPECT_DOUBLE_EQ(MANIF_PI, std::abs(se2d.angle()));

  SE2f se2f = se2d.cast<float>();

  EXPECT_FLOAT_EQ(4, se2f.x());
  EXPECT_FLOAT_EQ(2, se2f.y());
  EXPECT_FLOAT_EQ(MANIF_PI, std::abs(se2f.angle()));
}

TEST(TEST_SE2, TEST_SE2_MAP_IDENTITY)
{
  double data[4] = {4,2,1,0};
  Eigen::Map<SE2d> se2(data);

  se2.setIdentity();

  EXPECT_DOUBLE_EQ(0, se2.x());
  EXPECT_DOUBLE_EQ(0, se2.y());
  EXPECT_DOUBLE_EQ(0, se2.angle());

  EXPECT_DOUBLE_EQ(0, data[0]);
  EXPECT_DOUBLE_EQ(0, data[1]);
  EXPECT_DOUBLE_EQ(1, data[2]);
  EXPECT_DOUBLE_EQ(0, data[3]);
}

TEST(TEST_SE2, TEST_SE2_MAP_IDENTITY2)
{
  double data[4] = {4,2,1,0};
  Eigen::Map<SE2d> se2(data);
  se2 = SE2d::Identity();

  EXPECT_DOUBLE_EQ(0, se2.x());
  EXPECT_DOUBLE_EQ(0, se2.y());
  EXPECT_DOUBLE_EQ(0, se2.angle());

  EXPECT_DOUBLE_EQ(0, data[0]);
  EXPECT_DOUBLE_EQ(0, data[1]);
  EXPECT_DOUBLE_EQ(1, data[2]);
  EXPECT_DOUBLE_EQ(0, data[3]);
}

TEST(TEST_SE2, TEST_SE2_MAP_RANDOM)
{
  double data[4] = {4,2,1,0};
  Eigen::Map<SE2d> se2(data);

  se2.setRandom();

  const auto complex = se2.coeffs().block<2,1>(2,0);

  EXPECT_DOUBLE_EQ(1, complex.norm());
}

TEST(TEST_SE2, TEST_SE2_MAP_RANDOM2)
{
  double data[4] = {4,2,1,0};
  Eigen::Map<SE2d> se2(data);
  se2 = SE2d::Random();

  const auto complex = se2.coeffs().block<2,1>(2,0);

  EXPECT_DOUBLE_EQ(1, complex.norm());
}

TEST(TEST_SE2, TEST_SE2_MAP_MATRIX)
{
  double data[4] = {4,2,1,0};
  Eigen::Map<SE2d> se2(data);

  SE2d::Transformation t = se2.transform();

  EXPECT_EQ(3, t.rows());
  EXPECT_EQ(3, t.cols());

  /// @todo Eigen matrix comparison
}

TEST(TEST_SE2, TEST_SE2_MAP_ROTATION)
{
  double data[4] = {4,2,1,0};
  Eigen::Map<SE2d> se2(data);

  SE2d::Rotation r = se2.rotation();

  EXPECT_EQ(2, r.rows());
  EXPECT_EQ(2, r.cols());

  /// @todo Eigen matrix comparison
}

TEST(TEST_SE2, TEST_SE2_MAP_ASSIGN_OP)
{
  double dataa[4] = {4,2,1,0};
  Eigen::Map<SE2d> se2a(dataa);

  double datab[4] = {-4,-2,-1,0};
  Eigen::Map<SE2d>se2b(datab);

  se2a = se2b;

  EXPECT_DOUBLE_EQ(-4, se2a.x());
  EXPECT_DOUBLE_EQ(-2, se2a.y());
  EXPECT_ANGLE_NEAR(-MANIF_PI, se2a.angle(), 1e-15);

  EXPECT_DOUBLE_EQ(datab[0], dataa[0]);
  EXPECT_DOUBLE_EQ(datab[1], dataa[1]);
  EXPECT_DOUBLE_EQ(datab[2], dataa[2]);
  EXPECT_DOUBLE_EQ(datab[3], dataa[3]);
}

TEST(TEST_SE2, TEST_SE2_MAP_INVERSE)
{
  double data[4] = {0,0,1,0};
  Eigen::Map<SE2d> se2(data);

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
  EXPECT_ANGLE_NEAR(-MANIF_PI, se2_inv.angle(), 1e-15);
  EXPECT_DOUBLE_EQ(-1, se2_inv.real());
  EXPECT_NEAR(0, se2_inv.imag(), 1e-15);

  se2 = SE2d(0.7, 2.3, MANIF_PI/3.);
  se2_inv = se2.inverse();

  EXPECT_DOUBLE_EQ(-2.341858428704209, se2_inv.x());
  EXPECT_DOUBLE_EQ(-0.543782217350893, se2_inv.y());
  EXPECT_ANGLE_NEAR(-1.04719755119660, se2_inv.angle(), 3e-15);
}

TEST(TEST_SE2, TEST_SE2_MAP_RPLUS_ZERO)
{
  double data[4] = {1,1,-1,0};
  Eigen::Map<SE2d> se2(data);

  SE2Tangentd se2t(0, 0, 0);

  auto se2c = se2.rplus(se2t);

  EXPECT_DOUBLE_EQ(1, se2c.x());
  EXPECT_DOUBLE_EQ(1, se2c.y());
  EXPECT_ANGLE_NEAR(-MANIF_PI, se2c.angle(), 1e-15);
}
/*
TEST(TEST_SE2, TEST_SE2_MAP_RPLUS)
{
  Eigen::Map<SE2d> se2a(1, 1, MANIF_PI / 2.);
  SE2Tangentd se2b(1, 1, MANIF_PI / 2.);

  auto se2c = se2a.rplus(se2b);

  /// @todo what to expect here ?? :S
//  EXPECT_DOUBLE_EQ(0, se2c.x());
//  EXPECT_DOUBLE_EQ(2, se2c.y());
  EXPECT_DOUBLE_EQ(MANIF_PI, se2c.angle());
}

TEST(TEST_SE2, TEST_SE2_MAP_LPLUS_ZERO)
{
  Eigen::Map<SE2d>se2a(1, 1, MANIF_PI / 2.);
  SE2Tangentd se2b(0, 0, 0);

  auto se2c = se2a.lplus(se2b);

  EXPECT_DOUBLE_EQ(1, se2c.x());
  EXPECT_DOUBLE_EQ(1, se2c.y());
  EXPECT_DOUBLE_EQ(MANIF_PI / 2., se2c.angle());
}

TEST(TEST_SE2, TEST_SE2_MAP_LPLUS)
{
  Eigen::Map<SE2d>se2a(1, 1, MANIF_PI / 2.);
  SE2Tangentd se2b(1, 1, MANIF_PI / 2.);

  auto se2c = se2a.lplus(se2b);

  /// @todo what to expect here ?? :S
//  EXPECT_DOUBLE_EQ(1, se2c.x());
//  EXPECT_DOUBLE_EQ(1, se2c.y());
  EXPECT_DOUBLE_EQ(MANIF_PI, se2c.angle());
}

TEST(TEST_SE2, TEST_SE2_MAP_PLUS)
{
  Eigen::Map<SE2d>se2a(1, 1, MANIF_PI / 2.);
  SE2Tangentd se2b(1, 1, MANIF_PI / 2.);

  auto se2c = se2a.plus(se2b);

  /// @todo what to expect here ?? :S
//  EXPECT_DOUBLE_EQ(0, se2c.x());
//  EXPECT_DOUBLE_EQ(2, se2c.y());
  EXPECT_DOUBLE_EQ(MANIF_PI, se2c.angle());
}

TEST(TEST_SE2, TEST_SE2_MAP_OP_PLUS)
{
  Eigen::Map<SE2d>se2a(1, 1, MANIF_PI / 2.);
  SE2Tangentd se2b(1, 1, MANIF_PI / 2.);

  auto se2c = se2a + se2b;

  /// @todo what to expect here ?? :S
//  EXPECT_DOUBLE_EQ(0, se2c.x());
//  EXPECT_DOUBLE_EQ(2, se2c.y());
  EXPECT_DOUBLE_EQ(MANIF_PI, se2c.angle());
}

TEST(TEST_SE2, TEST_SE2_MAP_OP_PLUS_EQ)
{
  Eigen::Map<SE2d>se2a(1, 1, MANIF_PI / 2.);
  SE2Tangentd se2b(1, 1, MANIF_PI / 2.);

  se2a += se2b;

  /// @todo what to expect here ?? :S
//  EXPECT_DOUBLE_EQ(0, se2a.x());
//  EXPECT_DOUBLE_EQ(2, se2a.y());
  EXPECT_DOUBLE_EQ(MANIF_PI, se2a.angle());
}

TEST(TEST_SE2, TEST_SE2_MAP_RMINUS_ZERO)
{
  Eigen::Map<SE2d>se2a(0, 0, 0);
  Eigen::Map<SE2d>se2b(0, 0, 0);

  auto se2c = se2a.rminus(se2b);

//  EXPECT_DOUBLE_EQ(0, se2c.x());
  EXPECT_NEAR(0, se2c.x(), 1e-15);
//  EXPECT_DOUBLE_EQ(0, se2c.y());
  EXPECT_NEAR(0, se2c.y(), 1e-15);
  EXPECT_DOUBLE_EQ(0, se2c.angle());
}

TEST(TEST_SE2, TEST_SE2_MAP_RMINUS_I)
{
  Eigen::Map<SE2d>se2a(1, 1, MANIF_PI);
  Eigen::Map<SE2d>se2b(1, 1, MANIF_PI);

  auto se2c = se2a.rminus(se2b);

  /// @todo
//  EXPECT_DOUBLE_EQ(0, se2c.x());
//  EXPECT_NEAR(0, se2c.x(), 1e-15);
//  EXPECT_DOUBLE_EQ(0, se2c.y());
//  EXPECT_NEAR(0, se2c.y(), 1e-15);
  EXPECT_DOUBLE_EQ(0, se2c.angle());
}

TEST(TEST_SE2, TEST_SE2_MAP_RMINUS)
{
  Eigen::Map<SE2d>se2a(1, 1, MANIF_PI);
  Eigen::Map<SE2d>se2b(2, 2, MANIF_PI_2);

  auto se2c = se2a.rminus(se2b);

  /// @todo
//  EXPECT_DOUBLE_EQ(1, se2c.x());
//  EXPECT_DOUBLE_EQ(1, se2c.y());
  EXPECT_DOUBLE_EQ(MANIF_PI_2, se2c.angle());
}

TEST(TEST_SE2, TEST_SE2_MAP_LMINUS_IDENTITY)
{
  Eigen::Map<SE2d>se2a(0,0,0);
  Eigen::Map<SE2d>se2b(0,0,0);

  auto se2c = se2a.lminus(se2b);

  /// @todo
  EXPECT_DOUBLE_EQ(0, se2c.x());
  EXPECT_DOUBLE_EQ(0, se2c.y());
  EXPECT_DOUBLE_EQ(0, se2c.angle());
}

TEST(TEST_SE2, TEST_SE2_MAP_LMINUS)
{
  Eigen::Map<SE2d>se2a(1,1,MANIF_PI);
  Eigen::Map<SE2d>se2b(2,2,MANIF_PI_2);

  auto se2c = se2a.lminus(se2b);

  /// @todo
//  EXPECT_DOUBLE_EQ(0, se2c.x());
//  EXPECT_DOUBLE_EQ(0, se2c.y());
  EXPECT_DOUBLE_EQ(-MANIF_PI_2, se2c.angle());
}

TEST(TEST_SE2, TEST_SE2_MAP_MINUS)
{
  Eigen::Map<SE2d>se2a(1, 1, MANIF_PI);
  Eigen::Map<SE2d>se2b(2, 2, MANIF_PI_2);

  auto se2c = se2a.minus(se2b);

  /// @todo
//  EXPECT_DOUBLE_EQ(1, se2c.x());
//  EXPECT_DOUBLE_EQ(1, se2c.y());
  EXPECT_DOUBLE_EQ(MANIF_PI_2, se2c.angle());
}

TEST(TEST_SE2, TEST_SE2_MAP_LIFT)
{
  Eigen::Map<SE2d>se2(1,1,MANIF_PI);

  auto se2_log = se2.log();

  /// @todo
//  EXPECT_DOUBLE_EQ(1, se2_log.x());
//  EXPECT_DOUBLE_EQ(1, se2_log.y());
  EXPECT_DOUBLE_EQ(MANIF_PI, se2_log.angle());
}

TEST(TEST_SE2, TEST_SE2_MAP_COMPOSE)
{
  Eigen::Map<SE2d>se2a(1,1,MANIF_PI_2);
  Eigen::Map<SE2d>se2b(2,2,MANIF_PI_2);

  auto se2c = se2a.compose(se2b);

  EXPECT_DOUBLE_EQ(-1, se2c.x());
  EXPECT_DOUBLE_EQ(+3, se2c.y());
  EXPECT_DOUBLE_EQ(MANIF_PI, se2c.angle());
}

TEST(TEST_SE2, TEST_SE2_MAP_OP_COMPOSE)
{
  Eigen::Map<SE2d>se2a(1,1,MANIF_PI_2);
  Eigen::Map<SE2d>se2b(2,2,MANIF_PI_2);

  auto se2c = se2a * se2b;

  EXPECT_DOUBLE_EQ(-1, se2c.x());
  EXPECT_DOUBLE_EQ(+3, se2c.y());
  EXPECT_DOUBLE_EQ(MANIF_PI, se2c.angle());
}

TEST(TEST_SE2, TEST_SE2_MAP_OP_COMPOSE_EQ)
{
  Eigen::Map<SE2d>se2a(1,1,MANIF_PI_2);
  Eigen::Map<SE2d>se2b(2,2,MANIF_PI_2);

  se2a *= se2b;

  EXPECT_DOUBLE_EQ(-1, se2a.x());
  EXPECT_DOUBLE_EQ(+3, se2a.y());
  EXPECT_DOUBLE_EQ(MANIF_PI, se2a.angle());
}

TEST(TEST_SE2, TEST_SE2_MAP_BETWEEN_I)
{
  Eigen::Map<SE2d>se2a(1,1,MANIF_PI);
  Eigen::Map<SE2d>se2b(1,1,MANIF_PI);

  auto se2c = se2a.between(se2b);

  EXPECT_DOUBLE_EQ(0, se2c.x());
  EXPECT_DOUBLE_EQ(0, se2c.y());
  EXPECT_DOUBLE_EQ(0, se2c.angle());
}

TEST(TEST_SE2, TEST_SE2_MAP_BETWEEN)
{
  Eigen::Map<SE2d>se2a(1,1,MANIF_PI);
  Eigen::Map<SE2d>se2b(2,2,MANIF_PI_2);

  auto se2c = se2a.between(se2b);

  EXPECT_DOUBLE_EQ(-1, se2c.x());
  EXPECT_DOUBLE_EQ(-1, se2c.y());
  EXPECT_DOUBLE_EQ(-MANIF_PI_2, se2c.angle());
}
*/

/*
/// with Jacs

TEST(TEST_SE2, TEST_SE2_MAP_INVERSE_JAC)
{
  Eigen::Map<SE2d>se2 = SE2d::Identity();

  Eigen::Map<SE2d>se2_inv;
  SE2d::Jacobian J_inv;
  se2.inverse(se2_inv, J_inv);

  EXPECT_DOUBLE_EQ(se2.angle(), se2_inv.angle());
  EXPECT_DOUBLE_EQ(1, se2_inv.real());
  EXPECT_DOUBLE_EQ(0, se2_inv.imag());

  EXPECT_EQ(1, J_inv.rows());
  EXPECT_EQ(1, J_inv.cols());
  EXPECT_DOUBLE_EQ(-1, J_inv(0));

  se2 = SE2d(MANIF_PI);
  se2.inverse(se2_inv, J_inv);

  EXPECT_DOUBLE_EQ(-MANIF_PI, se2_inv.angle());

  EXPECT_EQ(1, J_inv.rows());
  EXPECT_EQ(1, J_inv.cols());
  EXPECT_DOUBLE_EQ(-1, J_inv(0));
}

TEST(TEST_SE2, TEST_SE2_MAP_LIFT_JAC)
{
  Eigen::Map<SE2d>se2(MANIF_PI);

  SE2d::Tangent se2_log;
  SE2d::Tangent::Jacobian J_log;

  se2.log(se2_log, J_log);

  EXPECT_DOUBLE_EQ(MANIF_PI, se2_log.angle());

  /// @todo check this J
  EXPECT_EQ(1, J_log.rows());
  EXPECT_EQ(1, J_log.cols());
  EXPECT_DOUBLE_EQ(1, J_log(0));
}

TEST(TEST_SE2, TEST_SE2_MAP_COMPOSE_JAC)
{
  Eigen::Map<SE2d>se2a(MANIF_PI_2);
  Eigen::Map<SE2d>se2b(MANIF_PI_2);

  Eigen::Map<SE2d>se2c;
  SE2d::Jacobian J_c_a, J_c_b;

  se2a.compose(se2b, se2c, J_c_a, J_c_b);

  EXPECT_DOUBLE_EQ(MANIF_PI, se2c.angle());

  EXPECT_EQ(1, J_c_a.rows());
  EXPECT_EQ(1, J_c_a.cols());
  EXPECT_DOUBLE_EQ(1, J_c_a(0));

  EXPECT_EQ(1, J_c_b.rows());
  EXPECT_EQ(1, J_c_b.cols());
  EXPECT_DOUBLE_EQ(1, J_c_b(0));
}

TEST(TEST_SE2, TEST_SE2_MAP_RPLUS_JAC)
{
  Eigen::Map<SE2d>se2a(MANIF_PI / 2.);
  SE2Tangentd se2b(MANIF_PI / 2.);

  Eigen::Map<SE2d>se2c;
  SE2d::Jacobian J_rplus_m;
  SE2d::Jacobian J_rplus_t;

  se2a.rplus(se2b, se2c, J_rplus_m, J_rplus_t);

  EXPECT_DOUBLE_EQ(MANIF_PI, se2c.angle());

  EXPECT_EQ(1, J_rplus_m.rows());
  EXPECT_EQ(1, J_rplus_m.cols());
  EXPECT_DOUBLE_EQ(1, J_rplus_m(0));

  EXPECT_EQ(1, J_rplus_t.rows());
  EXPECT_EQ(1, J_rplus_t.cols());
  EXPECT_DOUBLE_EQ(1, J_rplus_t(0));
}

TEST(TEST_SE2, TEST_SE2_MAP_LPLUS_JAC)
{
  Eigen::Map<SE2d>se2a(MANIF_PI / 2.);
  SE2Tangentd se2b(MANIF_PI / 2.);

  Eigen::Map<SE2d>se2c;
  SE2d::Jacobian J_lplus_t;
  SE2d::Jacobian J_lplus_m;

  se2a.lplus(se2b, se2c, J_lplus_t, J_lplus_m);

  EXPECT_DOUBLE_EQ(MANIF_PI, se2c.angle());

  EXPECT_EQ(1, J_lplus_t.rows());
  EXPECT_EQ(1, J_lplus_t.cols());
  EXPECT_DOUBLE_EQ(1, J_lplus_t(0));

  EXPECT_EQ(1, J_lplus_m.rows());
  EXPECT_EQ(1, J_lplus_m.cols());
  EXPECT_DOUBLE_EQ(1, J_lplus_m(0));
}

TEST(TEST_SE2, TEST_SE2_MAP_PLUS_JAC)
{
  Eigen::Map<SE2d>se2a(MANIF_PI / 2.);
  SE2Tangentd se2b(MANIF_PI / 2.);

  Eigen::Map<SE2d>se2c;
  SE2d::Jacobian J_plus_m;
  SE2d::Jacobian J_plus_t;

  se2a.plus(se2b, se2c, J_plus_m, J_plus_t);

  EXPECT_DOUBLE_EQ(MANIF_PI, se2c.angle());

  EXPECT_EQ(1, J_plus_m.rows());
  EXPECT_EQ(1, J_plus_m.cols());
  EXPECT_DOUBLE_EQ(1, J_plus_m(0));

  EXPECT_EQ(1, J_plus_t.rows());
  EXPECT_EQ(1, J_plus_t.cols());
  EXPECT_DOUBLE_EQ(1, J_plus_t(0));
}

TEST(TEST_SE2, TEST_SE2_MAP_RMINUS_JAC)
{
  Eigen::Map<SE2d>se2a(MANIF_PI);
  Eigen::Map<SE2d>se2b(MANIF_PI_2);

  SE2Tangentd se2c;

  SE2d::Jacobian J_rminus_a, J_rminus_b;

  se2a.rminus(se2b, se2c, J_rminus_a, J_rminus_b);

  EXPECT_DOUBLE_EQ(MANIF_PI_2, se2c.angle());

  EXPECT_EQ(1, J_rminus_a.rows());
  EXPECT_EQ(1, J_rminus_a.cols());
  EXPECT_DOUBLE_EQ(1, J_rminus_a(0));

  EXPECT_EQ(1, J_rminus_b.rows());
  EXPECT_EQ(1, J_rminus_b.cols());
  EXPECT_DOUBLE_EQ(-1, J_rminus_b(0));
}

TEST(TEST_SE2, TEST_SE2_MAP_LMINUS_JAC)
{
  Eigen::Map<SE2d>se2a(MANIF_PI);
  Eigen::Map<SE2d>se2b(MANIF_PI_2);

  SE2Tangentd se2c;

  SE2d::Jacobian J_lminus_a, J_lminus_b;

  se2a.lminus(se2b, se2c, J_lminus_a, J_lminus_b);

  EXPECT_DOUBLE_EQ(-MANIF_PI_2, se2c.angle());

  EXPECT_EQ(1, J_lminus_a.rows());
  EXPECT_EQ(1, J_lminus_a.cols());
  EXPECT_DOUBLE_EQ(-1, J_lminus_a(0));

  EXPECT_EQ(1, J_lminus_b.rows());
  EXPECT_EQ(1, J_lminus_b.cols());
  EXPECT_DOUBLE_EQ(1, J_lminus_b(0));
}

TEST(TEST_SE2, TEST_SE2_MAP_MINUS_JAC)
{
  Eigen::Map<SE2d>se2a(MANIF_PI);
  Eigen::Map<SE2d>se2b(MANIF_PI_2);

  SE2Tangentd se2c;

  SE2d::Jacobian J_minus_a, J_minus_b;

  se2a.minus(se2b, se2c, J_minus_a, J_minus_b);

  EXPECT_DOUBLE_EQ(MANIF_PI_2, se2c.angle());

  EXPECT_EQ(1, J_minus_a.rows());
  EXPECT_EQ(1, J_minus_a.cols());
  EXPECT_DOUBLE_EQ(1, J_minus_a(0));

  EXPECT_EQ(1, J_minus_b.rows());
  EXPECT_EQ(1, J_minus_b.cols());
  EXPECT_DOUBLE_EQ(-1, J_minus_b(0));
}

TEST(TEST_SE2, TEST_SE2_MAP_BETWEEN_JAC)
{
  Eigen::Map<SE2d>se2a(MANIF_PI);
  Eigen::Map<SE2d>se2b(MANIF_PI_2);

  SE2d::Jacobian J_between_a, J_between_b;
  Eigen::Map<SE2d>se2c;

  se2a.between(se2b, se2c, J_between_a, J_between_b);

  EXPECT_DOUBLE_EQ(-MANIF_PI_2, se2c.angle());

  EXPECT_EQ(1, J_between_a.rows());
  EXPECT_EQ(1, J_between_a.cols());
  EXPECT_DOUBLE_EQ(-1, J_between_a(0));

  EXPECT_EQ(1, J_between_b.rows());
  EXPECT_EQ(1, J_between_b.cols());
  EXPECT_DOUBLE_EQ(1, J_between_b(0));
}
*/

MANIF_RUN_ALL_TEST;
