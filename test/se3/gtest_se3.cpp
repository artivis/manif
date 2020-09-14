#include <gtest/gtest.h>

#include "manif/SE3.h"
#include "../common_tester.h"

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

TEST(TEST_SE3, TEST_SE3_CONSTRUCTOR_T_AA)
{
  SE3d se3(SE3d::Translation(1,2,3),
           Eigen::AngleAxis<double>(Eigen::Quaterniond::Identity()));

  EXPECT_DOUBLE_EQ(1, se3.coeffs()(0));
  EXPECT_DOUBLE_EQ(2, se3.coeffs()(1));
  EXPECT_DOUBLE_EQ(3, se3.coeffs()(2));
  EXPECT_DOUBLE_EQ(0, se3.coeffs()(3));
  EXPECT_DOUBLE_EQ(0, se3.coeffs()(4));
  EXPECT_DOUBLE_EQ(0, se3.coeffs()(5));
  EXPECT_DOUBLE_EQ(1, se3.coeffs()(6));
}

TEST(TEST_SE3, TEST_SE3_CONSTRUCTOR_ISOMETRY)
{
  Eigen::Isometry3d h = Eigen::Translation3d(1,2,3) * Eigen::Quaterniond::Identity();

  SE3d se3(h);

  EXPECT_DOUBLE_EQ(1, se3.coeffs()(0));
  EXPECT_DOUBLE_EQ(2, se3.coeffs()(1));
  EXPECT_DOUBLE_EQ(3, se3.coeffs()(2));
  EXPECT_DOUBLE_EQ(0, se3.coeffs()(3));
  EXPECT_DOUBLE_EQ(0, se3.coeffs()(4));
  EXPECT_DOUBLE_EQ(0, se3.coeffs()(5));
  EXPECT_DOUBLE_EQ(1, se3.coeffs()(6));

  EXPECT_EIGEN_NEAR(h.matrix(), se3.transform());
  EXPECT_EIGEN_NEAR(h.matrix(), se3.isometry().matrix());
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

TEST(TEST_SE3, TEST_SE3_SET_AXIS_ANGLE)
{
  SE3d::DataType values; values << 0,0,0, 0,0,0,1;
  SE3d se3(values);

  Eigen::AngleAxis<double> axisAngle(0.23, Eigen::Vector3d::UnitZ());
  Eigen::Quaterniond quat(axisAngle);

  se3.quat(axisAngle);

  EXPECT_DOUBLE_EQ(0, se3.coeffs()(0));
  EXPECT_DOUBLE_EQ(0, se3.coeffs()(1));
  EXPECT_DOUBLE_EQ(0, se3.coeffs()(2));
  EXPECT_DOUBLE_EQ(quat.coeffs()(0), se3.coeffs()(3));
  EXPECT_DOUBLE_EQ(quat.coeffs()(1), se3.coeffs()(4));
  EXPECT_DOUBLE_EQ(quat.coeffs()(2), se3.coeffs()(5));
  EXPECT_DOUBLE_EQ(quat.coeffs()(3), se3.coeffs()(6));
}

TEST(TEST_SE3, TEST_SE3_SET_QUATERNION)
{
  SE3d::DataType values; values << 0,0,0, 0,0,0,1;
  SE3d se3(values);

  Eigen::AngleAxis<double> axisAngle(0.23, Eigen::Vector3d::UnitZ());
  Eigen::Quaterniond quat(axisAngle);

  se3.quat(quat);

  EXPECT_DOUBLE_EQ(0, se3.coeffs()(0));
  EXPECT_DOUBLE_EQ(0, se3.coeffs()(1));
  EXPECT_DOUBLE_EQ(0, se3.coeffs()(2));
  EXPECT_DOUBLE_EQ(quat.coeffs()(0), se3.coeffs()(3));
  EXPECT_DOUBLE_EQ(quat.coeffs()(1), se3.coeffs()(4));
  EXPECT_DOUBLE_EQ(quat.coeffs()(2), se3.coeffs()(5));
  EXPECT_DOUBLE_EQ(quat.coeffs()(3), se3.coeffs()(6));
}

TEST(TEST_SE3, TEST_SE3_SET_SO3)
{
  SE3d::DataType values; values << 0,0,0, 0,0,0,1;
  SE3d se3(values);
  SO3d so3(Eigen::AngleAxis<double>(0.23, Eigen::Vector3d::UnitZ()));

  se3.quat(so3);

  EXPECT_DOUBLE_EQ(0, se3.coeffs()(0));
  EXPECT_DOUBLE_EQ(0, se3.coeffs()(1));
  EXPECT_DOUBLE_EQ(0, se3.coeffs()(2));
  EXPECT_DOUBLE_EQ(so3.coeffs()(0), se3.coeffs()(3));
  EXPECT_DOUBLE_EQ(so3.coeffs()(1), se3.coeffs()(4));
  EXPECT_DOUBLE_EQ(so3.coeffs()(2), se3.coeffs()(5));
  EXPECT_DOUBLE_EQ(so3.coeffs()(3), se3.coeffs()(6));
}

TEST(TEST_SE3, TEST_SE3_SET_TRANSLATION)
{
  SE3d::DataType values; values << 0,0,0, 0,0,0,1;
  SE3d se3(values);

  se3.translation(Eigen::Vector3d(1,1,1));

  EXPECT_DOUBLE_EQ(1, se3.coeffs()(0));
  EXPECT_DOUBLE_EQ(1, se3.coeffs()(1));
  EXPECT_DOUBLE_EQ(1, se3.coeffs()(2));
  EXPECT_DOUBLE_EQ(0, se3.coeffs()(3));
  EXPECT_DOUBLE_EQ(0, se3.coeffs()(4));
  EXPECT_DOUBLE_EQ(0, se3.coeffs()(5));
  EXPECT_DOUBLE_EQ(1, se3.coeffs()(6));
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

  se3 = SE3d::Random();

  Eigen::Isometry3d iso(se3.asSO3().quat());
  iso.translation() = se3.translation();

  EXPECT_EIGEN_NEAR(iso.matrix(), se3.transform());
  EXPECT_EIGEN_NEAR(iso.inverse().matrix(), se3.inverse().transform());
  EXPECT_EIGEN_NEAR(Eigen::Matrix4d::Identity(), (se3.inverse()*se3).transform());

  EXPECT_MANIF_NEAR(SE3d::Identity(), se3.inverse()*se3);
  EXPECT_MANIF_NEAR(SE3d::Identity(), se3*se3.inverse());
}

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

TEST(TEST_SE3, TEST_SE3_ACT)
{
  SE3d se3 = SE3d::Identity();

  auto transformed_point = se3.act(Eigen::Vector3d(1,1,1));

  EXPECT_NEAR(+1, transformed_point.x(), 1e-15);
  EXPECT_NEAR(+1, transformed_point.y(), 1e-15);
  EXPECT_NEAR(+1, transformed_point.z(), 1e-15);

  se3 = SE3d(1,1,1,MANIF_PI,MANIF_PI_2,MANIF_PI/4.);

  transformed_point = se3.act(Eigen::Vector3d(1,1,1));

  EXPECT_NEAR( 1, transformed_point.x(), 1e-15);
  EXPECT_NEAR(-0.414213562373, transformed_point.y(), 1e-12);
  EXPECT_NEAR( 0, transformed_point.z(), 1e-15);

  se3 = SE3d(-1,-1,-1,MANIF_PI/4,-MANIF_PI_2,-MANIF_PI);

  transformed_point = se3.act(Eigen::Vector3d(1,1,1));

  EXPECT_NEAR( 0.414213562373, transformed_point.x(), 1e-12);
  EXPECT_NEAR(-1, transformed_point.y(), 1e-15);
  EXPECT_NEAR( 0, transformed_point.z(), 1e-15);
}

TEST(TEST_SE3, TEST_SE3_TRANSFORM)
{
  Eigen::Isometry3d h = Eigen::Translation3d(1,2,3) * Eigen::Quaterniond::Identity();

  SE3d se3(h);

  Eigen::Matrix4d se3h = se3.transform();

  EXPECT_DOUBLE_EQ(1, se3h(0,0));
  EXPECT_DOUBLE_EQ(0, se3h(0,1));
  EXPECT_DOUBLE_EQ(0, se3h(0,2));
  EXPECT_DOUBLE_EQ(1, se3h(0,3));
  EXPECT_DOUBLE_EQ(0, se3h(1,0));
  EXPECT_DOUBLE_EQ(1, se3h(1,1));
  EXPECT_DOUBLE_EQ(0, se3h(1,2));
  EXPECT_DOUBLE_EQ(2, se3h(1,3));
  EXPECT_DOUBLE_EQ(0, se3h(2,0));
  EXPECT_DOUBLE_EQ(0, se3h(2,1));
  EXPECT_DOUBLE_EQ(1, se3h(2,2));
  EXPECT_DOUBLE_EQ(3, se3h(2,3));
}

TEST(TEST_SE3, TEST_SE3_ISOMETRY)
{
  Eigen::Isometry3d h = Eigen::Translation3d(1,2,3) * Eigen::Quaterniond::Identity();

  SE3d se3(h);

  Eigen::Isometry3d se3h = se3.isometry();

  EXPECT_DOUBLE_EQ(1, se3h.matrix()(0,0));
  EXPECT_DOUBLE_EQ(0, se3h.matrix()(0,1));
  EXPECT_DOUBLE_EQ(0, se3h.matrix()(0,2));
  EXPECT_DOUBLE_EQ(1, se3h.matrix()(0,3));
  EXPECT_DOUBLE_EQ(0, se3h.matrix()(1,0));
  EXPECT_DOUBLE_EQ(1, se3h.matrix()(1,1));
  EXPECT_DOUBLE_EQ(0, se3h.matrix()(1,2));
  EXPECT_DOUBLE_EQ(2, se3h.matrix()(1,3));
  EXPECT_DOUBLE_EQ(0, se3h.matrix()(2,0));
  EXPECT_DOUBLE_EQ(0, se3h.matrix()(2,1));
  EXPECT_DOUBLE_EQ(1, se3h.matrix()(2,2));
  EXPECT_DOUBLE_EQ(3, se3h.matrix()(2,3));
}

#ifndef MANIF_NO_DEBUG

TEST(TEST_SE3, TEST_SE3_CONSTRUCTOR_NOT_NORMALIZED_ARGS)
{
  // EXPECT_THROW(
  //   SE3d se3(SE3d(1, 1)),
  //   manif::invalid_argument
  // );

  SE3d::DataType values; values << 0,0,0, 1,1,1,1;

  EXPECT_THROW(
    SE3d se3(values),
    manif::invalid_argument
  );

  try {
    SE3d se3(values);
  } catch (manif::invalid_argument& e) {
    EXPECT_FALSE(std::string(e.what()).empty());
  }
}

TEST(TEST_SE3, TEST_SE3_CONSTRUCTOR_UNNORMALIZED)
{
  using DataType = typename SE3d::DataType;
  EXPECT_THROW(
    SE3d(DataType::Random()*10.), manif::invalid_argument
  );
}

TEST(TEST_SE3, TEST_SE3_NORMALIZE)
{
  using DataType = SE3d::DataType;
  DataType data = DataType::Random() * 100.;

  EXPECT_THROW(
    SE3d a(data), manif::invalid_argument
  );

  Eigen::Map<SE3d> map(data.data());
  map.normalize();

  EXPECT_NO_THROW({
    SE3d b = map
    (void)b;
  });
}

#endif

MANIF_TEST(SE3d);

MANIF_TEST_MAP(SE3d);

MANIF_TEST_JACOBIANS(SE3d);

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
