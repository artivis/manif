#include "manif/SO2.h"

#include "../common_tester.h"

using namespace manif;

TEST(TEST_SO2, TEST_SO2_PROPS)
{
  EXPECT_EQ(2, SO2d::Dim);
  EXPECT_EQ(1, SO2d::DoF);
  EXPECT_EQ(2, SO2d::RepSize);
}

TEST(TEST_SO2, TEST_SO2_CONSTRUCTOR_DATATYPE)
{
  SO2d so2(SO2d::DataType(1,0));

  EXPECT_DOUBLE_EQ(0, so2.angle());
}

TEST(TEST_SO2, TEST_SO2_CONSTRUCTOR_REAL_IMAG)
{
  SO2d so2(1, 0);

  EXPECT_DOUBLE_EQ(0, so2.angle());
}

TEST(TEST_SO2, TEST_SO2_CONSTRUCTOR_THETA)
{
  SO2d so2(0);

  EXPECT_DOUBLE_EQ(0, so2.angle());
}

TEST(TEST_SO2, TEST_SO2_CONSTRUCTOR_COPY)
{
  SO2d so2(SO2d(1,1));

  EXPECT_DOUBLE_EQ(M_PI/4., so2.angle());
}

TEST(TEST_SO2, TEST_SO2_COEFFS)
{
  SO2d so2(0);

  EXPECT_DOUBLE_EQ(1, so2.coeffs()(0));
  EXPECT_DOUBLE_EQ(0, so2.coeffs()(1));
}

TEST(TEST_SO2, TEST_SO2_DATA)
{
  SO2d so2(0);

  double * data_ptr = so2.data();

  ASSERT_NE(nullptr, data_ptr);

  EXPECT_DOUBLE_EQ(1, *data_ptr);
  ++data_ptr;
  EXPECT_DOUBLE_EQ(0, *data_ptr);
}

TEST(TEST_SO2, TEST_SO2_CAST)
{
  SO2d so2d(0);

  EXPECT_DOUBLE_EQ(0, so2d.angle());

  SO2f so2f = so2d.cast<float>();

  EXPECT_FLOAT_EQ(0, so2f.angle());
}

TEST(TEST_SO2, TEST_SO2_IDENTITY)
{
  SO2d so2;

  so2.setIdentity();

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

  so2.setRandom();

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

  EXPECT_EIGEN_NEAR(Eigen::Matrix3d::Identity(), t);
}

TEST(TEST_SO2, TEST_SO2_ROTATION)
{
  SO2d so2 = SO2d::Identity();

  SO2d::Rotation r = so2.rotation();

  EXPECT_EQ(2, r.rows());
  EXPECT_EQ(2, r.cols());

  EXPECT_EIGEN_NEAR(Eigen::Matrix2d::Identity(), r);
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

TEST(TEST_SO2, TEST_SO2_RMINUS2)
{
  SO2d so2a(3.*M_PI/8.);
  SO2d so2b(0);

  auto so2c = so2a.rminus(so2b);

  EXPECT_DOUBLE_EQ(3.*M_PI/8., so2c.angle());
}

TEST(TEST_SO2, TEST_SO2_LMINUS)
{
  SO2d so2a(M_PI);
  SO2d so2b(M_PI_2);

  auto so2c = so2a.lminus(so2b);

  EXPECT_DOUBLE_EQ(M_PI_2, so2c.angle());
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

  auto so2_log = so2.log();

  EXPECT_DOUBLE_EQ(M_PI, so2_log.angle());
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

  SO2d::Jacobian J_inv;
  SO2d so2_inv = so2.inverse(J_inv);

  EXPECT_DOUBLE_EQ(so2.angle(), so2_inv.angle());
  EXPECT_DOUBLE_EQ(1, so2_inv.real());
  EXPECT_DOUBLE_EQ(0, so2_inv.imag());

  EXPECT_EQ(1, J_inv.rows());
  EXPECT_EQ(1, J_inv.cols());
  EXPECT_DOUBLE_EQ(-1, J_inv(0));

  so2 = SO2d(M_PI);
  so2_inv = so2.inverse(J_inv);

  EXPECT_DOUBLE_EQ(-M_PI, so2_inv.angle());

  EXPECT_EQ(1, J_inv.rows());
  EXPECT_EQ(1, J_inv.cols());
  EXPECT_DOUBLE_EQ(-1, J_inv(0));
}

TEST(TEST_SO2, TEST_SO2_LIFT_JAC)
{
  SO2d so2(M_PI);

  SO2d::Tangent::Jacobian J_log;
  SO2d::Tangent so2_log = so2.log(J_log);

  EXPECT_DOUBLE_EQ(M_PI, so2_log.angle());

  /// @todo check this J
  EXPECT_EQ(1, J_log.rows());
  EXPECT_EQ(1, J_log.cols());
  EXPECT_DOUBLE_EQ(1, J_log(0));
}

TEST(TEST_SO2, TEST_SO2_COMPOSE_JAC)
{
  SO2d so2a(M_PI_2);
  SO2d so2b(M_PI_2);

  SO2d::Jacobian J_c_a, J_c_b;
  SO2d so2c = so2a.compose(so2b, J_c_a, J_c_b);

  so2c = so2a.compose(so2b, SO2d::_, J_c_b);

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

  SO2d::Jacobian J_rplus_m;
  SO2d::Jacobian J_rplus_t;

  SO2d so2c = so2a.rplus(so2b, J_rplus_m, J_rplus_t);

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

  SO2d::Jacobian J_lplus_t;
  SO2d::Jacobian J_lplus_m;

  SO2d so2c = so2a.lplus(so2b, J_lplus_t, J_lplus_m);

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

  SO2d::Jacobian J_plus_m;
  SO2d::Jacobian J_plus_t;

  SO2d so2c = so2a.plus(so2b, J_plus_m, J_plus_t);

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

  SO2d::Jacobian J_rminus_a, J_rminus_b;

  SO2Tangentd so2c = so2a.rminus(so2b, J_rminus_a, J_rminus_b);

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

  SO2d::Jacobian J_lminus_a, J_lminus_b;

  SO2Tangentd so2c = so2a.lminus(so2b, J_lminus_a, J_lminus_b);

  EXPECT_DOUBLE_EQ(M_PI_2, so2c.angle());

  EXPECT_EQ(1, J_lminus_a.rows());
  EXPECT_EQ(1, J_lminus_a.cols());
  EXPECT_DOUBLE_EQ(1, J_lminus_a(0));

  EXPECT_EQ(1, J_lminus_b.rows());
  EXPECT_EQ(1, J_lminus_b.cols());
  EXPECT_DOUBLE_EQ(-1, J_lminus_b(0));
}

TEST(TEST_SO2, TEST_SO2_MINUS_JAC)
{
  SO2d so2a(M_PI);
  SO2d so2b(M_PI_2);

  SO2d::Jacobian J_minus_a, J_minus_b;

  SO2Tangentd so2c = so2a.minus(so2b, J_minus_a, J_minus_b);

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
  SO2d so2c = so2a.between(so2b, J_between_a, J_between_b);

  EXPECT_DOUBLE_EQ(-M_PI_2, so2c.angle());

  EXPECT_EQ(1, J_between_a.rows());
  EXPECT_EQ(1, J_between_a.cols());
  EXPECT_DOUBLE_EQ(-1, J_between_a(0));

  EXPECT_EQ(1, J_between_b.rows());
  EXPECT_EQ(1, J_between_b.cols());
  EXPECT_DOUBLE_EQ(1, J_between_b(0));
}

TEST(TEST_SO2, TEST_SO2_ACT)
{
  SO2d so2(M_PI/2.);

  auto transformed_point = so2.act(Eigen::Vector2d(1,1));

  /// @todo precision issue ?
  //EXPECT_DOUBLE_EQ(0.0, transformed_point.x());
  //EXPECT_DOUBLE_EQ(0.0, transformed_point.y());

  EXPECT_NEAR(-1, transformed_point.x(), 1e-15);
  EXPECT_NEAR(+1, transformed_point.y(), 1e-15);

  so2 = SO2d(-M_PI/2.);

  transformed_point = so2.act(Eigen::Vector2d(1,1));

  EXPECT_NEAR(+1, transformed_point.x(), 1e-15);
  EXPECT_NEAR(-1, transformed_point.y(), 1e-15);

  so2 = SO2d::Identity();

  transformed_point = so2.act(Eigen::Vector2d(1,1));

  EXPECT_NEAR(+1, transformed_point.x(), 1e-15);
  EXPECT_NEAR(+1, transformed_point.y(), 1e-15);
}

MANIF_TEST(SO2d);

MANIF_TEST_JACOBIANS(SO2d);

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

//  ::testing::GTEST_FLAG(filter) = "TEST_SO2.TEST_SO2_LIFT_JAC";

  return RUN_ALL_TESTS();
}
