#include <gtest/gtest.h>

#include "manif/SO3.h"

#include "../common_tester.h"

using namespace manif;

TEST(TEST_SO3, TEST_SO3_CONSTRUCTOR_DATATYPE)
{
  SO3d so3(SO3d::DataType(0,0,0,1));

  EXPECT_DOUBLE_EQ(0, so3.x());
  EXPECT_DOUBLE_EQ(0, so3.y());
  EXPECT_DOUBLE_EQ(0, so3.z());
  EXPECT_DOUBLE_EQ(1, so3.w());
}

TEST(TEST_SO3, TEST_SO3_CONSTRUCTOR_QUAT)
{
  SO3d so3(Eigen::Quaterniond(1,0,0,0));

  EXPECT_DOUBLE_EQ(0, so3.x());
  EXPECT_DOUBLE_EQ(0, so3.y());
  EXPECT_DOUBLE_EQ(0, so3.z());
  EXPECT_DOUBLE_EQ(1, so3.w());
}

TEST(TEST_SO3, TEST_SO3_CONSTRUCTOR_QUAT_COEFFS)
{
  SO3d so3(0,0,0,1);

  EXPECT_DOUBLE_EQ(0, so3.x());
  EXPECT_DOUBLE_EQ(0, so3.y());
  EXPECT_DOUBLE_EQ(0, so3.z());
  EXPECT_DOUBLE_EQ(1, so3.w());
}

TEST(TEST_SO3, TEST_SO3_CONSTRUCTOR_ANGLE_AXIS)
{
  SO3d so3(Eigen::AngleAxis<double>(0, Eigen::Vector3d::UnitX()));

  EXPECT_DOUBLE_EQ(0, so3.x());
  EXPECT_DOUBLE_EQ(0, so3.y());
  EXPECT_DOUBLE_EQ(0, so3.z());
  EXPECT_DOUBLE_EQ(1, so3.w());
}

TEST(TEST_SO3, TEST_SO3_CONSTRUCTOR_ROLL_PITCH_YAW)
{
  SO3d so3(0, 0, 0);

  EXPECT_DOUBLE_EQ(0, so3.x());
  EXPECT_DOUBLE_EQ(0, so3.y());
  EXPECT_DOUBLE_EQ(0, so3.z());
  EXPECT_DOUBLE_EQ(1, so3.w());
}

TEST(TEST_SO3, TEST_SO3_CONSTRUCTOR_NOT_NORMALIZED_ARGS)
{
  EXPECT_THROW(
    SO3d so3(SO3d(1, 1, 1, 1)),
    manif::invalid_argument
  );

  EXPECT_THROW(
    SO3d so3(SO3d::DataType(1, 1, 1, 1)),
    manif::invalid_argument
  );

  try {
    SO3d so3(SO3d::DataType(1, 1, 1, 1));
  } catch (manif::invalid_argument& e) {
    EXPECT_FALSE(std::string(e.what()).empty());
  }
}

TEST(TEST_SO3, TEST_SO3_IDENTITY)
{
  SO3d so3;

  so3.setIdentity();

  EXPECT_DOUBLE_EQ(0, so3.x());
  EXPECT_DOUBLE_EQ(0, so3.y());
  EXPECT_DOUBLE_EQ(0, so3.z());
  EXPECT_DOUBLE_EQ(1, so3.w());
}

TEST(TEST_SO3, TEST_SO3_IDENTITY2)
{
  const SO3d so3 = SO3d::Identity();

  EXPECT_DOUBLE_EQ(0, so3.x());
  EXPECT_DOUBLE_EQ(0, so3.y());
  EXPECT_DOUBLE_EQ(0, so3.z());
  EXPECT_DOUBLE_EQ(1, so3.w());
}

TEST(TEST_SO3, TEST_SO3_RANDOM)
{
  SO3d so3;

  so3.setRandom();

  const SO3d& so3_ref = so3;

  EXPECT_DOUBLE_EQ(1, so3_ref.coeffs().norm());
}

TEST(TEST_SO3, TEST_SO3_RANDOM2)
{
  const SO3d so3 = SO3d::Random();

  EXPECT_DOUBLE_EQ(1, so3.coeffs().norm());
}

TEST(TEST_SO3, TEST_SO3_TRANSFORM)
{
  SO3d so3 = SO3d::Identity();

  SO3d::Transformation t = so3.transform();

  EXPECT_EQ(4, t.rows());
  EXPECT_EQ(4, t.cols());

  EXPECT_DOUBLE_EQ(1, t(0,0));
  EXPECT_DOUBLE_EQ(0, t(0,1));
  EXPECT_DOUBLE_EQ(0, t(0,2));
  EXPECT_DOUBLE_EQ(0, t(0,3));
  EXPECT_DOUBLE_EQ(0, t(1,0));
  EXPECT_DOUBLE_EQ(1, t(1,1));
  EXPECT_DOUBLE_EQ(0, t(1,2));
  EXPECT_DOUBLE_EQ(0, t(1,3));
  EXPECT_DOUBLE_EQ(0, t(2,0));
  EXPECT_DOUBLE_EQ(0, t(2,1));
  EXPECT_DOUBLE_EQ(1, t(2,2));
  EXPECT_DOUBLE_EQ(0, t(2,3));
  EXPECT_DOUBLE_EQ(0, t(3,0));
  EXPECT_DOUBLE_EQ(0, t(3,1));
  EXPECT_DOUBLE_EQ(0, t(3,2));
  EXPECT_DOUBLE_EQ(1, t(3,3));

  /// @todo Eigen matrix comparison
}

TEST(TEST_SO3, TEST_SO3_ROTATION)
{
  SO3d so3 = SO3d::Identity();

  SO3d::Rotation r = so3.rotation();

  EXPECT_EQ(3, r.rows());
  EXPECT_EQ(3, r.cols());

  EXPECT_DOUBLE_EQ(1, r(0,0));
  EXPECT_DOUBLE_EQ(0, r(0,1));
  EXPECT_DOUBLE_EQ(0, r(0,2));
  EXPECT_DOUBLE_EQ(0, r(1,0));
  EXPECT_DOUBLE_EQ(1, r(1,1));
  EXPECT_DOUBLE_EQ(0, r(1,2));
  EXPECT_DOUBLE_EQ(0, r(2,0));
  EXPECT_DOUBLE_EQ(0, r(2,1));
  EXPECT_DOUBLE_EQ(1, r(2,2));

  /// @todo Eigen matrix comparison
}

TEST(TEST_SO3, TEST_SO3_ASSIGN_OP)
{
  SO3d so3a = SO3d::Random();
  SO3d so3b = SO3d::Random();

  so3a = so3b;

  EXPECT_DOUBLE_EQ(so3a.x(), so3b.x());
  EXPECT_DOUBLE_EQ(so3a.y(), so3b.y());
  EXPECT_DOUBLE_EQ(so3a.z(), so3b.z());
  EXPECT_DOUBLE_EQ(so3a.w(), so3b.w());
}


TEST(TEST_SO3, TEST_SO3_INVERSE)
{
    // inverse of identity is identity
  SO3d so3 = SO3d::Identity();

  auto so3_inv = so3.inverse();

  EXPECT_DOUBLE_EQ(0, so3_inv.x());
  EXPECT_DOUBLE_EQ(0, so3_inv.y());
  EXPECT_DOUBLE_EQ(0, so3_inv.z());
  EXPECT_DOUBLE_EQ(1, so3_inv.w());

  // inverse of random in quaternion form is conjugate
  so3 = SO3d::Random();

  so3_inv = so3.inverse();

  EXPECT_DOUBLE_EQ(so3.x(), -so3_inv.x());
  EXPECT_DOUBLE_EQ(so3.y(), -so3_inv.y());
  EXPECT_DOUBLE_EQ(so3.z(), -so3_inv.z());
  EXPECT_DOUBLE_EQ(so3.w(), +so3_inv.w());
}

TEST(TEST_SO3, TEST_SO3_RPLUS)
{
    // Adding zero to Identity
  SO3d so3a = SO3d::Identity();
  SO3Tangentd so3b = SO3Tangentd::Zero();

  auto so3c = so3a.rplus(so3b);

  EXPECT_DOUBLE_EQ(0, so3c.x());
  EXPECT_DOUBLE_EQ(0, so3c.y());
  EXPECT_DOUBLE_EQ(0, so3c.z());
  EXPECT_DOUBLE_EQ(1, so3c.w());

  // Adding zero to random
  so3a = SO3d::Random();

  so3c = so3a.rplus(so3b);

  EXPECT_DOUBLE_EQ(so3a.x(), so3c.x());
  EXPECT_DOUBLE_EQ(so3a.y(), so3c.y());
  EXPECT_DOUBLE_EQ(so3a.z(), so3c.z());
  EXPECT_DOUBLE_EQ(so3a.w(), so3c.w());

  // todo: adding something to something
}

TEST(TEST_SO3, TEST_SO3_LPLUS)
{
    // Adding zero to Identity
    SO3d so3a = SO3d::Identity();
    SO3Tangentd so3t = SO3Tangentd::Zero();

    auto so3c = so3a.lplus(so3t);

    EXPECT_DOUBLE_EQ(0, so3c.x());
    EXPECT_DOUBLE_EQ(0, so3c.y());
    EXPECT_DOUBLE_EQ(0, so3c.z());
    EXPECT_DOUBLE_EQ(1, so3c.w());

    // Adding zero to random
    so3a = SO3d::Random();

    so3c = so3a.lplus(so3t);

    EXPECT_DOUBLE_EQ(so3a.x(), so3c.x());
    EXPECT_DOUBLE_EQ(so3a.y(), so3c.y());
    EXPECT_DOUBLE_EQ(so3a.z(), so3c.z());
    EXPECT_DOUBLE_EQ(so3a.w(), so3c.w());

    // todo: adding something to something
}

TEST(TEST_SO3, TEST_SO3_PLUS)
{
    // plus() is the same as rplus()
  SO3d so3a = SO3d::Random();
  SO3Tangentd so3t = SO3Tangentd::Random();

  auto so3c = so3a.plus(so3t);
  auto so3d = so3a.rplus(so3t);

  EXPECT_DOUBLE_EQ(so3c.x(), so3d.x());
  EXPECT_DOUBLE_EQ(so3c.y(), so3d.y());
  EXPECT_DOUBLE_EQ(so3c.z(), so3d.z());
  EXPECT_DOUBLE_EQ(so3c.w(), so3d.w());
}

TEST(TEST_SO3, TEST_SO3_OP_PLUS)
{
    // manif + tangent is the same as rplus()
  SO3d so3a = SO3d::Random();
  SO3Tangentd so3t = SO3Tangentd::Random();

  auto so3c = so3a + so3t;
  auto so3d = so3a.rplus(so3t);

  EXPECT_DOUBLE_EQ(so3c.x(), so3d.x());
  EXPECT_DOUBLE_EQ(so3c.y(), so3d.y());
  EXPECT_DOUBLE_EQ(so3c.z(), so3d.z());
  EXPECT_DOUBLE_EQ(so3c.w(), so3d.w());
}

TEST(TEST_SO3, TEST_SO3_OP_PLUS_EQ)
{
    // manif += tangent is the same as rplus()
  SO3d so3a = SO3d::Random();
  SO3Tangentd so3t = SO3Tangentd::Random();

  auto so3d = so3a.rplus(so3t);
  so3a += so3t;

  EXPECT_DOUBLE_EQ(so3a.x(), so3d.x());
  EXPECT_DOUBLE_EQ(so3a.y(), so3d.y());
  EXPECT_DOUBLE_EQ(so3a.z(), so3d.z());
  EXPECT_DOUBLE_EQ(so3a.w(), so3d.w());
}

TEST(TEST_SO3, TEST_SO3_RMINUS)
{
    // identity minus identity is zero
  SO3d so3a = SO3d::Identity();
  SO3d so3b = SO3d::Identity();

  auto so3c = so3a.rminus(so3b);

  EXPECT_DOUBLE_EQ(0, so3c.coeffs()(0));
  EXPECT_DOUBLE_EQ(0, so3c.coeffs()(1));
  EXPECT_DOUBLE_EQ(0, so3c.coeffs()(2));

  // random minus the same is zero
  so3a = SO3d::Random();
  so3b = so3a;

  so3c = so3a.rminus(so3b);

  EXPECT_NEAR(0, so3c.coeffs()(0), 1e-15);
  EXPECT_NEAR(0, so3c.coeffs()(1), 1e-15);
  EXPECT_NEAR(0, so3c.coeffs()(2), 1e-15);

  // todo subtracting something from something
}

TEST(TEST_SO3, TEST_SO3_LMINUS)
{
    // identity minus identity is zero
  SO3d so3a = SO3d::Identity();
  SO3d so3b = SO3d::Identity();

  auto so3c = so3a.rminus(so3b);

  EXPECT_DOUBLE_EQ(0, so3c.coeffs()(0));
  EXPECT_DOUBLE_EQ(0, so3c.coeffs()(1));
  EXPECT_DOUBLE_EQ(0, so3c.coeffs()(2));

  // random minus the same is zero
  so3a = SO3d::Random();
  so3b = so3a;

  so3c = so3a.rminus(so3b);

  EXPECT_NEAR(0, so3c.coeffs()(0), 1e-15);
  EXPECT_NEAR(0, so3c.coeffs()(1), 1e-15);
  EXPECT_NEAR(0, so3c.coeffs()(2), 1e-15);

  // todo subtracting something from something
}

TEST(TEST_SO3, TEST_SO3_MINUS)
{
    // minus is the same as rminus
  SO3d so3a = SO3d::Random();
  SO3d so3b = SO3d::Random();

  auto so3c = so3a.minus(so3b);
  auto so3d = so3a.rminus(so3b);

  EXPECT_DOUBLE_EQ(so3d.coeffs()(0), so3c.coeffs()(0));
  EXPECT_DOUBLE_EQ(so3d.coeffs()(1), so3c.coeffs()(1));
  EXPECT_DOUBLE_EQ(so3d.coeffs()(2), so3c.coeffs()(2));
}

TEST(TEST_SO3, TEST_SO3_OP_MINUS)
{
    // '-' is the same as rminus, and in the same order
  SO3d so3a = SO3d::Random();
  SO3d so3b = SO3d::Random();

  auto so3c = so3a - so3b;
  auto so3d = so3a.rminus(so3b);

  EXPECT_DOUBLE_EQ(so3d.coeffs()(0), so3c.coeffs()(0));
  EXPECT_DOUBLE_EQ(so3d.coeffs()(1), so3c.coeffs()(1));
  EXPECT_DOUBLE_EQ(so3d.coeffs()(2), so3c.coeffs()(2));
}

TEST(TEST_SO3, TEST_SO3_RETRACT)
{
    // exp of zero is identity
    SO3Tangentd so3t = SO3Tangentd::Zero();

    auto so3 = so3t.exp();

    EXPECT_DOUBLE_EQ(0, so3.x());
    EXPECT_DOUBLE_EQ(0, so3.y());
    EXPECT_DOUBLE_EQ(0, so3.z());
    EXPECT_DOUBLE_EQ(1, so3.w());

    // exp of negative is inverse of exp, that is, its conjugate
    so3t = SO3Tangentd::Random(); // something
    so3 = so3t.exp(); // exp of something

    SO3Tangentd so3n(-so3t.coeffs()); // minus something
    auto so3_inv = so3n.exp(); // exp of minus something

    EXPECT_DOUBLE_EQ(so3_inv.x(), -so3.x()); // check conjugate
    EXPECT_DOUBLE_EQ(so3_inv.y(), -so3.y());
    EXPECT_DOUBLE_EQ(so3_inv.z(), -so3.z());
    EXPECT_DOUBLE_EQ(so3_inv.w(), +so3.w());

}

TEST(TEST_SO3, TEST_SO3_LIFT)
{
    // Lift of Identity is Zero
  SO3d so3 = SO3d::Identity();

  auto so3_log = so3.log();

  EXPECT_DOUBLE_EQ(0, so3_log.coeffs()(0));
  EXPECT_DOUBLE_EQ(0, so3_log.coeffs()(1));
  EXPECT_DOUBLE_EQ(0, so3_log.coeffs()(2));

  // Lift of inverse is minus log
  so3 = SO3d::Random();
  so3_log = so3.log();

  auto so3_inv_log = so3.inverse().log();

  EXPECT_DOUBLE_EQ(so3_inv_log.coeffs()(0), -so3_log.coeffs()(0));
  EXPECT_DOUBLE_EQ(so3_inv_log.coeffs()(1), -so3_log.coeffs()(1));
  EXPECT_DOUBLE_EQ(so3_inv_log.coeffs()(2), -so3_log.coeffs()(2));
}

/// with Jacs

TEST(TEST_SO3, TEST_SO3_INVERSE_JAC)
{
    // Inverse of identity is identity; Jac is minus identity
  SO3d so3 = SO3d::Identity();

  SO3d::Jacobian J_inv;
  SO3d so3_inv = so3.inverse(J_inv);

  EXPECT_DOUBLE_EQ(0, so3_inv.x());
  EXPECT_DOUBLE_EQ(0, so3_inv.y());
  EXPECT_DOUBLE_EQ(0, so3_inv.z());
  EXPECT_DOUBLE_EQ(1, so3_inv.w());

  EXPECT_EQ(3, J_inv.rows());
  EXPECT_EQ(3, J_inv.cols());

  EXPECT_EQ(-1, J_inv(0,0));
  EXPECT_EQ( 0, J_inv(0,1));
  EXPECT_EQ( 0, J_inv(0,2));
  EXPECT_EQ( 0, J_inv(1,0));
  EXPECT_EQ(-1, J_inv(1,1));
  EXPECT_EQ( 0, J_inv(1,2));
  EXPECT_EQ( 0, J_inv(2,0));
  EXPECT_EQ( 0, J_inv(2,1));
  EXPECT_EQ(-1, J_inv(2,2));

  // Inverse of something is conjugate; Jac is minus rotation
  so3 = SO3d::Random();

  so3_inv = so3.inverse(J_inv);

  EXPECT_DOUBLE_EQ(-so3.x(), so3_inv.x());
  EXPECT_DOUBLE_EQ(-so3.y(), so3_inv.y());
  EXPECT_DOUBLE_EQ(-so3.z(), so3_inv.z());
  EXPECT_DOUBLE_EQ( so3.w(), so3_inv.w());

  EXPECT_EQ(3, J_inv.rows());
  EXPECT_EQ(3, J_inv.cols());

  EXPECT_EQ(-so3.rotation()(0,0), J_inv(0,0));
  EXPECT_EQ(-so3.rotation()(0,1), J_inv(0,1));
  EXPECT_EQ(-so3.rotation()(0,2), J_inv(0,2));
  EXPECT_EQ(-so3.rotation()(1,0), J_inv(1,0));
  EXPECT_EQ(-so3.rotation()(1,1), J_inv(1,1));
  EXPECT_EQ(-so3.rotation()(1,2), J_inv(1,2));
  EXPECT_EQ(-so3.rotation()(2,0), J_inv(2,0));
  EXPECT_EQ(-so3.rotation()(2,1), J_inv(2,1));
  EXPECT_EQ(-so3.rotation()(2,2), J_inv(2,2));
}

TEST(TEST_SO3, TEST_SO3_LIFT_JAC)
{
  SO3d so3(0,0,0); // Identity

  SO3d::Tangent::Jacobian J_log;

  /// @todo Jac not implemented yet
  SO3d::Tangent so3_log = so3.log(/*J_log*/);

  EXPECT_DOUBLE_EQ(0, so3_log.x());
  EXPECT_DOUBLE_EQ(0, so3_log.y());
  EXPECT_DOUBLE_EQ(0, so3_log.z());

  /// @todo check this J
  EXPECT_EQ(3, J_log.rows());
  EXPECT_EQ(3, J_log.cols());

//  EXPECT_DOUBLE_EQ(1, J_log(0));
}

TEST(TEST_SO3, TEST_SO3_RIGHT_LEFT_JAC_ADJ)
{
  SO3Tangentd tan = SO3Tangentd::Zero();
  EXPECT_EIGEN_NEAR(tan.ljac(), tan.exp().rotation()*tan.rjac());

  tan = SO3Tangentd::Random();
  EXPECT_EIGEN_NEAR(tan.ljac(), tan.exp().rotation()*tan.rjac());
}

TEST(TEST_SO3, TEST_SO3_RIGHT_LEFT_JAC)
{
  SO3Tangentd tan = SO3Tangentd::Zero();
  EXPECT_EIGEN_NEAR(tan.ljac(), tan.rjac().transpose());
  EXPECT_EIGEN_NEAR(tan.rjac(), tan.ljac().transpose());

  tan = SO3Tangentd::Random();
  EXPECT_EIGEN_NEAR(tan.ljac(), tan.rjac().transpose());
  EXPECT_EIGEN_NEAR(tan.rjac(), tan.ljac().transpose());
}

TEST(TEST_SO3, TEST_SO3_TANGENT_SKEW)
{
  SO3Tangentd so3_tan(SO3Tangentd::DataType(1, 2, 3));

  SO3Tangentd::LieAlg so3_lie = so3_tan.hat();

  EXPECT_DOUBLE_EQ( 0, so3_lie(0,0));
  EXPECT_DOUBLE_EQ(-3, so3_lie(0,1));
  EXPECT_DOUBLE_EQ( 2, so3_lie(0,2));
  EXPECT_DOUBLE_EQ( 3, so3_lie(1,0));
  EXPECT_DOUBLE_EQ( 0, so3_lie(1,1));
  EXPECT_DOUBLE_EQ(-1, so3_lie(1,2));
  EXPECT_DOUBLE_EQ(-2, so3_lie(2,0));
  EXPECT_DOUBLE_EQ( 1, so3_lie(2,1));
  EXPECT_DOUBLE_EQ( 0, so3_lie(2,2));
}

TEST(TEST_SO3, TEST_SO3_ACT)
{
  SO3d so3 = SO3d::Identity();

  auto transformed_point = so3.act(Eigen::Vector3d(1,1,1));

  /// @todo precision issue ?
  //EXPECT_DOUBLE_EQ(1, transformed_point.x());
  //EXPECT_DOUBLE_EQ(1, transformed_point.y());

  EXPECT_NEAR(+1, transformed_point.x(), 1e-15);
  EXPECT_NEAR(+1, transformed_point.y(), 1e-15);
  EXPECT_NEAR(+1, transformed_point.z(), 1e-15);

  so3 = SO3d(MANIF_PI, MANIF_PI_2, MANIF_PI/4.);

  transformed_point = so3.act(Eigen::Vector3d(1,1,1));

  EXPECT_NEAR( 0, transformed_point.x(), 1e-15);
  EXPECT_NEAR(-1.414213562373, transformed_point.y(), 1e-12);
  EXPECT_NEAR(-1, transformed_point.z(), 1e-15);

  so3 = SO3d(MANIF_PI/4, -MANIF_PI_2, -MANIF_PI);

  transformed_point = so3.act(Eigen::Vector3d(1,1,1));

  EXPECT_NEAR( 1.414213562373, transformed_point.x(), 1e-12);
  EXPECT_NEAR(-0, transformed_point.y(), 1e-15);
  EXPECT_NEAR( 1, transformed_point.z(), 1e-15);
}

TEST(TEST_SO3, TEST_SO3_CONSTRUCTOR_UNNORMALIZED)
{
  using DataType = typename SO3d::DataType;
  EXPECT_THROW(
    SO3d(DataType::Random()*10.), manif::invalid_argument
  );
}

TEST(TEST_SO3, TEST_SO3_NORMALIZE)
{
  using DataType = SO3d::DataType;
  DataType data = DataType::Random() * 100.;

  EXPECT_THROW(
    SO3d a(data), manif::invalid_argument
  );

  Eigen::Map<SO3d> map(data.data());
  map.normalize();

  EXPECT_NO_THROW(
    SO3d b = map
  );
}

MANIF_TEST(SO3d);

MANIF_TEST_JACOBIANS(SO3d);

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
