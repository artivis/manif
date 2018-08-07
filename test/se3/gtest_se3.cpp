#include <gtest/gtest.h>

#include "manif/SE3.h"
#include "../test_utils.h"

#include <Eigen/Geometry>

using namespace manif;

TEST(TEST_SE3, TEST_SE3_CONSTRUCTOR_DATATYPE)
{
  SE3d::DataType values; values << 0,0,0, 0,0,0,1;
  SE3d se3(values);

  EXPECT_DOUBLE_EQ(0, se3.coeffs()(0));
  EXPECT_DOUBLE_EQ(0, se3.coeffs()(1));
  EXPECT_DOUBLE_EQ(0, se3.coeffs()(2));
  EXPECT_DOUBLE_EQ(0, se3.coeffs()(3));
  EXPECT_DOUBLE_EQ(0, se3.coeffs()(4));
  EXPECT_DOUBLE_EQ(0, se3.coeffs()(5));
  EXPECT_DOUBLE_EQ(1, se3.coeffs()(6));
}

TEST(TEST_SE3, TEST_SE3_CONSTRUCTOR_T_Q)
{
  SE3d se3(SE3d::Translation(1,2,3),
           Eigen::Quaterniond::Identity());

  EXPECT_DOUBLE_EQ(1, se3.coeffs()(0));
  EXPECT_DOUBLE_EQ(2, se3.coeffs()(1));
  EXPECT_DOUBLE_EQ(3, se3.coeffs()(2));
  EXPECT_DOUBLE_EQ(0, se3.coeffs()(3));
  EXPECT_DOUBLE_EQ(0, se3.coeffs()(4));
  EXPECT_DOUBLE_EQ(0, se3.coeffs()(5));
  EXPECT_DOUBLE_EQ(1, se3.coeffs()(6));
}

TEST(TEST_SE3, TEST_SE3_CONSTRUCTOR_COPY)
{
  SE3d::DataType values; values << 0,0,0, 0,0,0,1;
  SE3d o(values);
  SE3d se3( o );

  EXPECT_DOUBLE_EQ(0, se3.coeffs()(0));
  EXPECT_DOUBLE_EQ(0, se3.coeffs()(1));
  EXPECT_DOUBLE_EQ(0, se3.coeffs()(2));
  EXPECT_DOUBLE_EQ(0, se3.coeffs()(3));
  EXPECT_DOUBLE_EQ(0, se3.coeffs()(4));
  EXPECT_DOUBLE_EQ(0, se3.coeffs()(5));
  EXPECT_DOUBLE_EQ(1, se3.coeffs()(6));
}

TEST(TEST_SE3, TEST_SE3_DATA)
{
  SE3d::DataType values; values << 0,0,0, 0,0,0,1;
  SE3d se3(values);

  double * data_ptr = se3.data();

  ASSERT_NE(nullptr, data_ptr);

  EXPECT_DOUBLE_EQ(0, *data_ptr);
  ++data_ptr;
  EXPECT_DOUBLE_EQ(0, *data_ptr);
  ++data_ptr;
  EXPECT_DOUBLE_EQ(0, *data_ptr);
  ++data_ptr;
  EXPECT_DOUBLE_EQ(0, *data_ptr);
  ++data_ptr;
  EXPECT_DOUBLE_EQ(0, *data_ptr);
  ++data_ptr;
  EXPECT_DOUBLE_EQ(0, *data_ptr);
  ++data_ptr;
  EXPECT_DOUBLE_EQ(1, *data_ptr);
}

TEST(TEST_SE3, TEST_SE3_CAST)
{
  SE3d se3d(SE3d::Translation(1,2,3),
            Eigen::Quaterniond::Identity());

  EXPECT_DOUBLE_EQ(1, se3d.coeffs()(0));
  EXPECT_DOUBLE_EQ(2, se3d.coeffs()(1));
  EXPECT_DOUBLE_EQ(3, se3d.coeffs()(2));
  EXPECT_DOUBLE_EQ(0, se3d.coeffs()(3));
  EXPECT_DOUBLE_EQ(0, se3d.coeffs()(4));
  EXPECT_DOUBLE_EQ(0, se3d.coeffs()(5));
  EXPECT_DOUBLE_EQ(1, se3d.coeffs()(6));

  SE3f se3f = se3d.cast<float>();

  EXPECT_DOUBLE_EQ(1, se3f.coeffs()(0));
  EXPECT_DOUBLE_EQ(2, se3f.coeffs()(1));
  EXPECT_DOUBLE_EQ(3, se3f.coeffs()(2));
  EXPECT_DOUBLE_EQ(0, se3f.coeffs()(3));
  EXPECT_DOUBLE_EQ(0, se3f.coeffs()(4));
  EXPECT_DOUBLE_EQ(0, se3f.coeffs()(5));
  EXPECT_DOUBLE_EQ(1, se3f.coeffs()(6));
}

TEST(TEST_SE3, TEST_SE3_IDENTITY)
{
  SE3d se3;

  se3.setIdentity();

  EXPECT_DOUBLE_EQ(0, se3.coeffs()(0));
  EXPECT_DOUBLE_EQ(0, se3.coeffs()(1));
  EXPECT_DOUBLE_EQ(0, se3.coeffs()(2));
  EXPECT_DOUBLE_EQ(0, se3.coeffs()(3));
  EXPECT_DOUBLE_EQ(0, se3.coeffs()(4));
  EXPECT_DOUBLE_EQ(0, se3.coeffs()(5));
  EXPECT_DOUBLE_EQ(1, se3.coeffs()(6));
}

TEST(TEST_SE3, TEST_SE3_IDENTITY2)
{
  SE3d se3 = SE3d::Identity();

  EXPECT_DOUBLE_EQ(0, se3.coeffs()(0));
  EXPECT_DOUBLE_EQ(0, se3.coeffs()(1));
  EXPECT_DOUBLE_EQ(0, se3.coeffs()(2));
  EXPECT_DOUBLE_EQ(0, se3.coeffs()(3));
  EXPECT_DOUBLE_EQ(0, se3.coeffs()(4));
  EXPECT_DOUBLE_EQ(0, se3.coeffs()(5));
  EXPECT_DOUBLE_EQ(1, se3.coeffs()(6));
}

//TEST(TEST_SE3, TEST_SE3_RANDOM)
//{
//  SE3d se3;

//  se3.setRandom();

//  const auto q_norm = se3.coeffs().block<4,1>(0,0).norm();

//  EXPECT_DOUBLE_EQ(1, q_norm);
//}

//TEST(TEST_SE3, TEST_SE3_RANDOM2)
//{
//  const SE3d se3 = SE3d::Random();

//  const auto q_norm = se3.coeffs().block<4,1>(0,0).norm();

//  EXPECT_DOUBLE_EQ(1, q_norm);
//}

TEST(TEST_SE3, TEST_SE3_MATRIX)
{
  SE3d se3 = SE3d::Identity();

  SE3d::Transformation t = se3.transform();

  EXPECT_EQ(4, t.rows());
  EXPECT_EQ(4, t.cols());

  /// @todo Eigen matrix comparison
}

TEST(TEST_SE3, TEST_SE3_ROTATION)
{
  SE3d se3 = SE3d::Identity();

  SE3d::Rotation r = se3.rotation();

  EXPECT_EQ(3, r.rows());
  EXPECT_EQ(3, r.cols());

  /// @todo Eigen matrix comparison
}

//TEST(TEST_SE3, TEST_SE3_ASSIGN_OP)
//{
//  SE3d se3a;
//  SE3d se3b = SE3d::Random();

//  se3a = se3b;

//  EXPECT_DOUBLE_EQ(se3b.coeffs()(0), se3a.coeffs()(0));
//  EXPECT_DOUBLE_EQ(se3b.coeffs()(1), se3a.coeffs()(1));
//  EXPECT_DOUBLE_EQ(se3b.coeffs()(2), se3a.coeffs()(2));
//  EXPECT_DOUBLE_EQ(se3b.coeffs()(3), se3a.coeffs()(3));
//  EXPECT_DOUBLE_EQ(se3b.coeffs()(4), se3a.coeffs()(4));
//  EXPECT_DOUBLE_EQ(se3b.coeffs()(5), se3a.coeffs()(5));
//  EXPECT_DOUBLE_EQ(se3b.coeffs()(6), se3a.coeffs()(6));
//}

TEST(TEST_SE3, TEST_SE3_INVERSE)
{
  SE3d se3 = SE3d::Identity();

  auto se3_inv = se3.inverse();

  EXPECT_DOUBLE_EQ(0, se3_inv.coeffs()(0));
  EXPECT_DOUBLE_EQ(0, se3_inv.coeffs()(1));
  EXPECT_DOUBLE_EQ(0, se3_inv.coeffs()(2));
  EXPECT_DOUBLE_EQ(0, se3_inv.coeffs()(3));
  EXPECT_DOUBLE_EQ(0, se3_inv.coeffs()(4));
  EXPECT_DOUBLE_EQ(0, se3_inv.coeffs()(5));
  EXPECT_DOUBLE_EQ(1, se3_inv.coeffs()(6));

//  SE3d::Jacobian J_minv_m;
//  se3_inv = se3.inverse(J_minv_m);

//  std::cout << "J_minv_m : \n" << J_minv_m << "\n";

//  se3 = SE3d(M_PI);
//  se3_inv = se3.inverse();

//  EXPECT_DOUBLE_EQ(-M_PI, se3_inv.angle());
}

/*
TEST(TEST_SE3, TEST_SE3_RPLUS)
{
  SE3d se3a(M_PI / 2.);
  SE3Tangentd se3b(M_PI / 2.);

  auto se3c = se3a.rplus(se3b);

  EXPECT_DOUBLE_EQ(M_PI, se3c.angle());
}

TEST(TEST_SE3, TEST_SE3_LPLUS)
{
  SE3d se3a(M_PI / 2.);
  SE3Tangentd se3b(M_PI / 2.);

  auto se3c = se3a.lplus(se3b);

  EXPECT_DOUBLE_EQ(M_PI, se3c.angle());
}

TEST(TEST_SE3, TEST_SE3_PLUS)
{
  SE3d se3a(M_PI / 2.);
  SE3Tangentd se3b(M_PI / 2.);

  auto se3c = se3a.plus(se3b);

  EXPECT_DOUBLE_EQ(M_PI, se3c.angle());
}

TEST(TEST_SE3, TEST_SE3_OP_PLUS)
{
  SE3d se3a(M_PI / 2.);
  SE3Tangentd se3b(M_PI / 2.);

  auto se3c = se3a + se3b;

  EXPECT_DOUBLE_EQ(M_PI, se3c.angle());
}

TEST(TEST_SE3, TEST_SE3_OP_PLUS_EQ)
{
  SE3d se3a(M_PI / 2.);
  SE3Tangentd se3b(M_PI / 2.);

  se3a += se3b;

  EXPECT_DOUBLE_EQ(M_PI, se3a.angle());
}

TEST(TEST_SE3, TEST_SE3_RMINUS)
{
  SE3d se3a(M_PI);
  SE3d se3b(M_PI_2);

  auto se3c = se3a.rminus(se3b);

  EXPECT_DOUBLE_EQ(M_PI_2, se3c.angle());
}

TEST(TEST_SE3, TEST_SE3_RMINUS2)
{
  SE3d se3a(3.*M_PI/8.);
  SE3d se3b(0);

  auto se3c = se3a.rminus(se3b);

  EXPECT_DOUBLE_EQ(3.*M_PI/8., se3c.angle());
}

TEST(TEST_SE3, TEST_SE3_LMINUS)
{
  SE3d se3a(M_PI);
  SE3d se3b(M_PI_2);

  auto se3c = se3a.lminus(se3b);

  EXPECT_DOUBLE_EQ(-M_PI_2, se3c.angle());
}

TEST(TEST_SE3, TEST_SE3_MINUS)
{
  SE3d se3a(M_PI);
  SE3d se3b(M_PI_2);

  auto se3c = se3a.minus(se3b);

  EXPECT_DOUBLE_EQ(M_PI_2, se3c.angle());
}

TEST(TEST_SE3, TEST_SE3_LIFT)
{
  SE3d se3(M_PI);

  auto se3_lift = se3.lift();

  EXPECT_DOUBLE_EQ(M_PI, se3_lift.angle());
}
*/

TEST(TEST_SE3, TEST_SE3_COMPOSE)
{
  SE3d se3a = SE3d::Identity();
  SE3d se3b = SE3d::Identity();

  auto se3c = se3a.compose(se3b);

  EXPECT_DOUBLE_EQ(0, se3c.coeffs()(0));
  EXPECT_DOUBLE_EQ(0, se3c.coeffs()(1));
  EXPECT_DOUBLE_EQ(0, se3c.coeffs()(2));
  EXPECT_DOUBLE_EQ(0, se3c.coeffs()(3));
  EXPECT_DOUBLE_EQ(0, se3c.coeffs()(4));
  EXPECT_DOUBLE_EQ(0, se3c.coeffs()(5));
  EXPECT_DOUBLE_EQ(1, se3c.coeffs()(6));
}

/*
TEST(TEST_SE3, TEST_SE3_OP_COMPOSE)
{
  SE3d se3a(M_PI_2);
  SE3d se3b(M_PI_2);

  auto se3c = se3a * se3b;

  EXPECT_DOUBLE_EQ(M_PI, se3c.angle());
}

TEST(TEST_SE3, TEST_SE3_OP_COMPOSE_EQ)
{
  SE3d se3a(M_PI_2);
  SE3d se3b(M_PI_2);

  se3a *= se3b;

  EXPECT_DOUBLE_EQ(M_PI, se3a.angle());
}

TEST(TEST_SE3, TEST_SE3_BETWEEN)
{
  SE3d se3a(M_PI);
  SE3d se3b(M_PI_2);

  auto se3c = se3a.between(se3b);

  EXPECT_DOUBLE_EQ(-M_PI_2, se3c.angle());
}

/// with Jacs

TEST(TEST_SE3, TEST_SE3_INVERSE_JAC)
{
  SE3d se3 = SE3d::Identity();

  SE3d::Jacobian J_inv;
  SE3d se3_inv = se3.inverse(J_inv);

  EXPECT_DOUBLE_EQ(se3.angle(), se3_inv.angle());
  EXPECT_DOUBLE_EQ(1, se3_inv.real());
  EXPECT_DOUBLE_EQ(0, se3_inv.imag());

  EXPECT_EQ(1, J_inv.rows());
  EXPECT_EQ(1, J_inv.cols());
  EXPECT_DOUBLE_EQ(-1, J_inv(0));

  se3 = SE3d(M_PI);
  se3_inv = se3.inverse(J_inv);

  EXPECT_DOUBLE_EQ(-M_PI, se3_inv.angle());

  EXPECT_EQ(1, J_inv.rows());
  EXPECT_EQ(1, J_inv.cols());
  EXPECT_DOUBLE_EQ(-1, J_inv(0));
}

TEST(TEST_SE3, TEST_SE3_LIFT_JAC)
{
  SE3d se3(M_PI);

  SE3d::Tangent::Jacobian J_lift;
  SE3d::Tangent se3_lift = se3.lift(J_lift);

  EXPECT_DOUBLE_EQ(M_PI, se3_lift.angle());

  /// @todo check this J
  EXPECT_EQ(1, J_lift.rows());
  EXPECT_EQ(1, J_lift.cols());
  EXPECT_DOUBLE_EQ(1, J_lift(0));
}

TEST(TEST_SE3, TEST_SE3_COMPOSE_JAC)
{
  SE3d se3a(M_PI_2);
  SE3d se3b(M_PI_2);

  SE3d::Jacobian J_c_a, J_c_b;
  SE3d se3c = se3a.compose(se3b, J_c_a, J_c_b);

  se3c = se3a.compose(se3b, SE3d::_, J_c_b);

  EXPECT_DOUBLE_EQ(M_PI, se3c.angle());

  EXPECT_EQ(1, J_c_a.rows());
  EXPECT_EQ(1, J_c_a.cols());
  EXPECT_DOUBLE_EQ(1, J_c_a(0));

  EXPECT_EQ(1, J_c_b.rows());
  EXPECT_EQ(1, J_c_b.cols());
  EXPECT_DOUBLE_EQ(1, J_c_b(0));
}

TEST(TEST_SE3, TEST_SE3_RPLUS_JAC)
{
  SE3d se3a(M_PI / 2.);
  SE3Tangentd se3b(M_PI / 2.);

  SE3d::Jacobian J_rplus_m;
  SE3d::Jacobian J_rplus_t;

  SE3d se3c = se3a.rplus(se3b, J_rplus_m, J_rplus_t);

  EXPECT_DOUBLE_EQ(M_PI, se3c.angle());

  EXPECT_EQ(1, J_rplus_m.rows());
  EXPECT_EQ(1, J_rplus_m.cols());
  EXPECT_DOUBLE_EQ(1, J_rplus_m(0));

  EXPECT_EQ(1, J_rplus_t.rows());
  EXPECT_EQ(1, J_rplus_t.cols());
  EXPECT_DOUBLE_EQ(1, J_rplus_t(0));
}

TEST(TEST_SE3, TEST_SE3_LPLUS_JAC)
{
  SE3d se3a(M_PI / 2.);
  SE3Tangentd se3b(M_PI / 2.);

  SE3d::Jacobian J_lplus_t;
  SE3d::Jacobian J_lplus_m;

  SE3d se3c = se3a.lplus(se3b, J_lplus_t, J_lplus_m);

  EXPECT_DOUBLE_EQ(M_PI, se3c.angle());

  EXPECT_EQ(1, J_lplus_t.rows());
  EXPECT_EQ(1, J_lplus_t.cols());
  EXPECT_DOUBLE_EQ(1, J_lplus_t(0));

  EXPECT_EQ(1, J_lplus_m.rows());
  EXPECT_EQ(1, J_lplus_m.cols());
  EXPECT_DOUBLE_EQ(1, J_lplus_m(0));
}

TEST(TEST_SE3, TEST_SE3_PLUS_JAC)
{
  SE3d se3a(M_PI / 2.);
  SE3Tangentd se3b(M_PI / 2.);

  SE3d::Jacobian J_plus_m;
  SE3d::Jacobian J_plus_t;

  SE3d se3c = se3a.plus(se3b, J_plus_m, J_plus_t);

  EXPECT_DOUBLE_EQ(M_PI, se3c.angle());

  EXPECT_EQ(1, J_plus_m.rows());
  EXPECT_EQ(1, J_plus_m.cols());
  EXPECT_DOUBLE_EQ(1, J_plus_m(0));

  EXPECT_EQ(1, J_plus_t.rows());
  EXPECT_EQ(1, J_plus_t.cols());
  EXPECT_DOUBLE_EQ(1, J_plus_t(0));
}

TEST(TEST_SE3, TEST_SE3_RMINUS_JAC)
{
  SE3d se3a(M_PI);
  SE3d se3b(M_PI_2);

  SE3d::Jacobian J_rminus_a, J_rminus_b;

  SE3Tangentd se3c = se3a.rminus(se3b, J_rminus_a, J_rminus_b);

  EXPECT_DOUBLE_EQ(M_PI_2, se3c.angle());

  EXPECT_EQ(1, J_rminus_a.rows());
  EXPECT_EQ(1, J_rminus_a.cols());
  EXPECT_DOUBLE_EQ(1, J_rminus_a(0));

  EXPECT_EQ(1, J_rminus_b.rows());
  EXPECT_EQ(1, J_rminus_b.cols());
  EXPECT_DOUBLE_EQ(-1, J_rminus_b(0));
}

TEST(TEST_SE3, TEST_SE3_LMINUS_JAC)
{
  SE3d se3a(M_PI);
  SE3d se3b(M_PI_2);

  SE3d::Jacobian J_lminus_a, J_lminus_b;

  SE3Tangentd se3c = se3a.lminus(se3b, J_lminus_a, J_lminus_b);

  EXPECT_DOUBLE_EQ(-M_PI_2, se3c.angle());

  EXPECT_EQ(1, J_lminus_a.rows());
  EXPECT_EQ(1, J_lminus_a.cols());
  EXPECT_DOUBLE_EQ(-1, J_lminus_a(0));

  EXPECT_EQ(1, J_lminus_b.rows());
  EXPECT_EQ(1, J_lminus_b.cols());
  EXPECT_DOUBLE_EQ(1, J_lminus_b(0));
}

TEST(TEST_SE3, TEST_SE3_MINUS_JAC)
{
  SE3d se3a(M_PI);
  SE3d se3b(M_PI_2);

  SE3d::Jacobian J_minus_a, J_minus_b;

  SE3Tangentd se3c = se3a.minus(se3b, J_minus_a, J_minus_b);

  EXPECT_DOUBLE_EQ(M_PI_2, se3c.angle());

  EXPECT_EQ(1, J_minus_a.rows());
  EXPECT_EQ(1, J_minus_a.cols());
  EXPECT_DOUBLE_EQ(1, J_minus_a(0));

  EXPECT_EQ(1, J_minus_b.rows());
  EXPECT_EQ(1, J_minus_b.cols());
  EXPECT_DOUBLE_EQ(-1, J_minus_b(0));
}

TEST(TEST_SE3, TEST_SE3_BETWEEN_JAC)
{
  SE3d se3a(M_PI);
  SE3d se3b(M_PI_2);

  SE3d::Jacobian J_between_a, J_between_b;
  SE3d se3c = se3a.between(se3b, J_between_a, J_between_b);

  EXPECT_DOUBLE_EQ(-M_PI_2, se3c.angle());

  EXPECT_EQ(1, J_between_a.rows());
  EXPECT_EQ(1, J_between_a.cols());
  EXPECT_DOUBLE_EQ(-1, J_between_a(0));

  EXPECT_EQ(1, J_between_b.rows());
  EXPECT_EQ(1, J_between_b.cols());
  EXPECT_DOUBLE_EQ(1, J_between_b(0));
}
*/

MANIF_TEST(SE3d);

MANIF_TEST_JACOBIANS(SE3d);

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

//  ::testing::GTEST_FLAG(filter) = "TEST_SE3.TEST_SE3_LIFT_JAC";

  return RUN_ALL_TESTS();
}
