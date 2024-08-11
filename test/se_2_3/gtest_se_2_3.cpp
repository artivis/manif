#include <gtest/gtest.h>

#include "manif/SE_2_3.h"
#include "../common_tester.h"

#include <Eigen/Geometry>
#include <iostream>
using namespace manif;

TEST(TEST_SE_2_3, TEST_SE_2_3_CONSTRUCTOR_DATATYPE)
{
  SE_2_3d::DataType values; values << 0,0,0, 0,0,0,1, 0,0,0;
  SE_2_3d se_2_3(values);

  EXPECT_DOUBLE_EQ(0, se_2_3.coeffs()(0));
  EXPECT_DOUBLE_EQ(0, se_2_3.coeffs()(1));
  EXPECT_DOUBLE_EQ(0, se_2_3.coeffs()(2));
  EXPECT_DOUBLE_EQ(0, se_2_3.coeffs()(3));
  EXPECT_DOUBLE_EQ(0, se_2_3.coeffs()(4));
  EXPECT_DOUBLE_EQ(0, se_2_3.coeffs()(5));
  EXPECT_DOUBLE_EQ(1, se_2_3.coeffs()(6));
  EXPECT_DOUBLE_EQ(0, se_2_3.coeffs()(7));
  EXPECT_DOUBLE_EQ(0, se_2_3.coeffs()(8));
  EXPECT_DOUBLE_EQ(0, se_2_3.coeffs()(9));
}

TEST(TEST_SE_2_3, TEST_SE_2_3_CONSTRUCTOR_T_Q_V)
{
  SE_2_3d se_2_3(SE_2_3d::Translation(1,2,3),
                 Eigen::Quaterniond::Identity(),
                 SE_2_3d::LinearVelocity(1, 2, 3));

  EXPECT_DOUBLE_EQ(1, se_2_3.coeffs()(0));
  EXPECT_DOUBLE_EQ(2, se_2_3.coeffs()(1));
  EXPECT_DOUBLE_EQ(3, se_2_3.coeffs()(2));
  EXPECT_DOUBLE_EQ(0, se_2_3.coeffs()(3));
  EXPECT_DOUBLE_EQ(0, se_2_3.coeffs()(4));
  EXPECT_DOUBLE_EQ(0, se_2_3.coeffs()(5));
  EXPECT_DOUBLE_EQ(1, se_2_3.coeffs()(6));
  EXPECT_DOUBLE_EQ(1, se_2_3.coeffs()(7));
  EXPECT_DOUBLE_EQ(2, se_2_3.coeffs()(8));
  EXPECT_DOUBLE_EQ(3, se_2_3.coeffs()(9));
}

TEST(TEST_SE_2_3, TEST_SE_2_3_CONSTRUCTOR_T_AA_V)
{
  SE_2_3d se_2_3(SE_2_3d::Translation(1,2,3),
                Eigen::AngleAxis<double>(Eigen::Quaterniond::Identity()),
                SE_2_3d::LinearVelocity(1, 2, 3));

  EXPECT_DOUBLE_EQ(1, se_2_3.coeffs()(0));
  EXPECT_DOUBLE_EQ(2, se_2_3.coeffs()(1));
  EXPECT_DOUBLE_EQ(3, se_2_3.coeffs()(2));
  EXPECT_DOUBLE_EQ(0, se_2_3.coeffs()(3));
  EXPECT_DOUBLE_EQ(0, se_2_3.coeffs()(4));
  EXPECT_DOUBLE_EQ(0, se_2_3.coeffs()(5));
  EXPECT_DOUBLE_EQ(1, se_2_3.coeffs()(6));
  EXPECT_DOUBLE_EQ(1, se_2_3.coeffs()(7));
  EXPECT_DOUBLE_EQ(2, se_2_3.coeffs()(8));
  EXPECT_DOUBLE_EQ(3, se_2_3.coeffs()(9));
}

TEST(TEST_SE_2_3, TEST_SE_2_3_CONSTRUCTOR_ISOMETRY)
{
  Eigen::Isometry3d h = Eigen::Translation3d(1,2,3) * Eigen::Quaterniond::Identity();
  SE_2_3d se_2_3(h, Eigen::Vector3d(1, 2, 3));

  Eigen::Matrix<double, 5, 5> hse23 = Eigen::Matrix<double, 5, 5>::Identity();
  hse23.topLeftCorner<4, 4>() = h.matrix();
  hse23.topRightCorner<3, 1>() = Eigen::Vector3d(1, 2, 3);

  EXPECT_DOUBLE_EQ(1, se_2_3.coeffs()(0));
  EXPECT_DOUBLE_EQ(2, se_2_3.coeffs()(1));
  EXPECT_DOUBLE_EQ(3, se_2_3.coeffs()(2));
  EXPECT_DOUBLE_EQ(0, se_2_3.coeffs()(3));
  EXPECT_DOUBLE_EQ(0, se_2_3.coeffs()(4));
  EXPECT_DOUBLE_EQ(0, se_2_3.coeffs()(5));
  EXPECT_DOUBLE_EQ(1, se_2_3.coeffs()(6));
  EXPECT_DOUBLE_EQ(1, se_2_3.coeffs()(7));
  EXPECT_DOUBLE_EQ(2, se_2_3.coeffs()(8));
  EXPECT_DOUBLE_EQ(3, se_2_3.coeffs()(9));
  EXPECT_EIGEN_NEAR(hse23, se_2_3.isometry());
}

TEST(TEST_SE_2_3, TEST_SE_2_3_CONSTRUCTOR_COPY)
{
  SE_2_3d::DataType values; values << 0,0,0, 0,0,0,1, 1,2,3;
  SE_2_3d o(values);
  SE_2_3d se_2_3( o );

  EXPECT_DOUBLE_EQ(0, se_2_3.coeffs()(0));
  EXPECT_DOUBLE_EQ(0, se_2_3.coeffs()(1));
  EXPECT_DOUBLE_EQ(0, se_2_3.coeffs()(2));
  EXPECT_DOUBLE_EQ(0, se_2_3.coeffs()(3));
  EXPECT_DOUBLE_EQ(0, se_2_3.coeffs()(4));
  EXPECT_DOUBLE_EQ(0, se_2_3.coeffs()(5));
  EXPECT_DOUBLE_EQ(1, se_2_3.coeffs()(6));
  EXPECT_DOUBLE_EQ(1, se_2_3.coeffs()(7));
  EXPECT_DOUBLE_EQ(2, se_2_3.coeffs()(8));
  EXPECT_DOUBLE_EQ(3, se_2_3.coeffs()(9));
}

TEST(TEST_SE_2_3, TEST_SE_2_3_DATA)
{
  SE_2_3d::DataType values; values << 0,0,0, 0,0,0,1, 0,0,0;
  SE_2_3d se_2_3(values);

  double * data_ptr = se_2_3.data();

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
  ++data_ptr;
  EXPECT_DOUBLE_EQ(0, *data_ptr);
  ++data_ptr;
  EXPECT_DOUBLE_EQ(0, *data_ptr);
  ++data_ptr;
  EXPECT_DOUBLE_EQ(0, *data_ptr);
}

TEST(TEST_SE_2_3, TEST_SE_2_3_CAST)
{
  SE_2_3d se_2_3d(SE_2_3d::Translation(1,2,3),
            Eigen::Quaterniond::Identity(),
            SE_2_3d::LinearVelocity(1,2,3));

  EXPECT_DOUBLE_EQ(1, se_2_3d.coeffs()(0));
  EXPECT_DOUBLE_EQ(2, se_2_3d.coeffs()(1));
  EXPECT_DOUBLE_EQ(3, se_2_3d.coeffs()(2));
  EXPECT_DOUBLE_EQ(0, se_2_3d.coeffs()(3));
  EXPECT_DOUBLE_EQ(0, se_2_3d.coeffs()(4));
  EXPECT_DOUBLE_EQ(0, se_2_3d.coeffs()(5));
  EXPECT_DOUBLE_EQ(1, se_2_3d.coeffs()(6));
  EXPECT_DOUBLE_EQ(1, se_2_3d.coeffs()(7));
  EXPECT_DOUBLE_EQ(2, se_2_3d.coeffs()(8));
  EXPECT_DOUBLE_EQ(3, se_2_3d.coeffs()(9));

  SE_2_3f se_2_3f = se_2_3d.cast<float>();

  EXPECT_DOUBLE_EQ(1, se_2_3f.coeffs()(0));
  EXPECT_DOUBLE_EQ(2, se_2_3f.coeffs()(1));
  EXPECT_DOUBLE_EQ(3, se_2_3f.coeffs()(2));
  EXPECT_DOUBLE_EQ(0, se_2_3f.coeffs()(3));
  EXPECT_DOUBLE_EQ(0, se_2_3f.coeffs()(4));
  EXPECT_DOUBLE_EQ(0, se_2_3f.coeffs()(5));
  EXPECT_DOUBLE_EQ(1, se_2_3f.coeffs()(6));
  EXPECT_DOUBLE_EQ(1, se_2_3f.coeffs()(7));
  EXPECT_DOUBLE_EQ(2, se_2_3f.coeffs()(8));
  EXPECT_DOUBLE_EQ(3, se_2_3f.coeffs()(9));
}

TEST(TEST_SE_2_3, TEST_SE_2_3_IDENTITY)
{
  SE_2_3d se_2_3;

  se_2_3.setIdentity();

  EXPECT_DOUBLE_EQ(0, se_2_3.coeffs()(0));
  EXPECT_DOUBLE_EQ(0, se_2_3.coeffs()(1));
  EXPECT_DOUBLE_EQ(0, se_2_3.coeffs()(2));
  EXPECT_DOUBLE_EQ(0, se_2_3.coeffs()(3));
  EXPECT_DOUBLE_EQ(0, se_2_3.coeffs()(4));
  EXPECT_DOUBLE_EQ(0, se_2_3.coeffs()(5));
  EXPECT_DOUBLE_EQ(1, se_2_3.coeffs()(6));
  EXPECT_DOUBLE_EQ(0, se_2_3.coeffs()(7));
  EXPECT_DOUBLE_EQ(0, se_2_3.coeffs()(8));
  EXPECT_DOUBLE_EQ(0, se_2_3.coeffs()(9));
}

TEST(TEST_SE_2_3, TEST_SE_2_3_IDENTITY2)
{
  SE_2_3d se_2_3 = SE_2_3d::Identity();

  EXPECT_DOUBLE_EQ(0, se_2_3.coeffs()(0));
  EXPECT_DOUBLE_EQ(0, se_2_3.coeffs()(1));
  EXPECT_DOUBLE_EQ(0, se_2_3.coeffs()(2));
  EXPECT_DOUBLE_EQ(0, se_2_3.coeffs()(3));
  EXPECT_DOUBLE_EQ(0, se_2_3.coeffs()(4));
  EXPECT_DOUBLE_EQ(0, se_2_3.coeffs()(5));
  EXPECT_DOUBLE_EQ(1, se_2_3.coeffs()(6));
  EXPECT_DOUBLE_EQ(0, se_2_3.coeffs()(7));
  EXPECT_DOUBLE_EQ(0, se_2_3.coeffs()(8));
  EXPECT_DOUBLE_EQ(0, se_2_3.coeffs()(9));
}

TEST(TEST_SE_2_3, TEST_SE_2_3_RANDOM)
{
 SE_2_3d se_2_3;

 se_2_3.setRandom();

 const auto q_norm = se_2_3.coeffs().segment<4>(3).norm();

 EXPECT_DOUBLE_EQ(1, q_norm);
}

TEST(TEST_SE_2_3, TEST_SE_2_3_RANDOM2)
{
 const SE_2_3d se_2_3 = SE_2_3d::Random();

 const auto q_norm = se_2_3.coeffs().segment<4>(3).norm();

 EXPECT_DOUBLE_EQ(1, q_norm);
}

TEST(TEST_SE_2_3, TEST_SE_2_3_MATRIX)
{
  SE_2_3d se_2_3 = SE_2_3d::Identity();

  SE_2_3d::Isometry t = se_2_3.isometry();

  EXPECT_EQ(5, t.rows());
  EXPECT_EQ(5, t.cols());

  /// @todo Eigen matrix comparison
}

TEST(TEST_SE_2_3, TEST_SE_2_3_ROTATION)
{
  SE_2_3d se_2_3 = SE_2_3d::Identity();

  SE_2_3d::Rotation r = se_2_3.rotation();

  EXPECT_EQ(3, r.rows());
  EXPECT_EQ(3, r.cols());

  /// @todo Eigen matrix comparison
}

TEST(TEST_SE_2_3, TEST_SE_2_3_INVERSE)
{
  SE_2_3d se_2_3 = SE_2_3d::Identity();

  auto se_2_3_inv = se_2_3.inverse();

  EXPECT_DOUBLE_EQ(0, se_2_3_inv.coeffs()(0));
  EXPECT_DOUBLE_EQ(0, se_2_3_inv.coeffs()(1));
  EXPECT_DOUBLE_EQ(0, se_2_3_inv.coeffs()(2));
  EXPECT_DOUBLE_EQ(0, se_2_3_inv.coeffs()(3));
  EXPECT_DOUBLE_EQ(0, se_2_3_inv.coeffs()(4));
  EXPECT_DOUBLE_EQ(0, se_2_3_inv.coeffs()(5));
  EXPECT_DOUBLE_EQ(1, se_2_3_inv.coeffs()(6));
  EXPECT_DOUBLE_EQ(0, se_2_3_inv.coeffs()(7));
  EXPECT_DOUBLE_EQ(0, se_2_3_inv.coeffs()(8));
  EXPECT_DOUBLE_EQ(0, se_2_3_inv.coeffs()(9));

  se_2_3 = SE_2_3d::Random();

  EXPECT_MANIF_NEAR(SE_2_3d::Identity(), se_2_3.inverse()*se_2_3);
  EXPECT_MANIF_NEAR(SE_2_3d::Identity(), se_2_3*se_2_3.inverse());
}

TEST(TEST_SE_2_3, TEST_SE_2_3_COMPOSE)
{
  SE_2_3d se_2_3a = SE_2_3d::Identity();
  SE_2_3d se_2_3b = SE_2_3d::Identity();

  auto se_2_3c = se_2_3a.compose(se_2_3b);

  EXPECT_DOUBLE_EQ(0, se_2_3c.coeffs()(0));
  EXPECT_DOUBLE_EQ(0, se_2_3c.coeffs()(1));
  EXPECT_DOUBLE_EQ(0, se_2_3c.coeffs()(2));
  EXPECT_DOUBLE_EQ(0, se_2_3c.coeffs()(3));
  EXPECT_DOUBLE_EQ(0, se_2_3c.coeffs()(4));
  EXPECT_DOUBLE_EQ(0, se_2_3c.coeffs()(5));
  EXPECT_DOUBLE_EQ(1, se_2_3c.coeffs()(6));
  EXPECT_DOUBLE_EQ(0, se_2_3c.coeffs()(7));
  EXPECT_DOUBLE_EQ(0, se_2_3c.coeffs()(8));
  EXPECT_DOUBLE_EQ(0, se_2_3c.coeffs()(9));
}

TEST(TEST_SE_2_3, TEST_SE_2_3_ACT)
{
  SE_2_3d se_2_3 = SE_2_3d::Identity();

  auto transformed_point = se_2_3.act(Eigen::Vector3d(1,1,1));

  EXPECT_NEAR(+1, transformed_point.x(), 1e-15);
  EXPECT_NEAR(+1, transformed_point.y(), 1e-15);
  EXPECT_NEAR(+1, transformed_point.z(), 1e-15);

  se_2_3 = SE_2_3d(1,1,1,MANIF_PI,MANIF_PI_2,MANIF_PI/4.,0,0,0);

  transformed_point = se_2_3.act(Eigen::Vector3d(1,1,1));

  EXPECT_NEAR( 1, transformed_point.x(), 1e-15);
  EXPECT_NEAR(-0.414213562373, transformed_point.y(), 1e-12);
  EXPECT_NEAR( 0, transformed_point.z(), 1e-15);

  // to demonstrate that the velocity terms do not affect the rigid motion action
  se_2_3 = SE_2_3d(-1,-1,-1,MANIF_PI/4,-MANIF_PI_2,-MANIF_PI,0,0,1);

  transformed_point = se_2_3.act(Eigen::Vector3d(1,1,1));

  EXPECT_NEAR( 0.414213562373, transformed_point.x(), 1e-12);
  EXPECT_NEAR(-1, transformed_point.y(), 1e-15);
  EXPECT_NEAR( 0, transformed_point.z(), 1e-15);
}

TEST(TEST_SE_2_3, TEST_SE_2_3TAN_LINANGVELACC)
{
  SE_2_3Tangentd::DataType data;
  data << 1,2,3,4,5,6,7,8,9;
  SE_2_3Tangentd se23tan(data);

  EXPECT_EIGEN_NEAR(Eigen::Vector3d(1,2,3),
                    se23tan.lin());

  EXPECT_EIGEN_NEAR(Eigen::Vector3d(4,5,6),
                    se23tan.ang());

  EXPECT_EIGEN_NEAR(Eigen::Vector3d(7,8,9),
                    se23tan.lin2());
}

#ifndef MANIF_NO_DEBUG

TEST(TEST_SE_2_3, TEST_SE_2_3_CONSTRUCTOR_NOT_NORMALIZED_ARGS)
{
  SE_2_3d::DataType values; values << 0,0,0, 1,1,1,1, 0,0,0;

  EXPECT_THROW(
    SE_2_3d se_2_3(values),
    manif::invalid_argument
  );

  try {
    SE_2_3d se_2_3(values);
  } catch (manif::invalid_argument& e) {
    EXPECT_FALSE(std::string(e.what()).empty());
  }
}

TEST(TEST_SE_2_3, TEST_SE_2_3_CONSTRUCTOR_UNNORMALIZED)
{
  using DataType = typename SE_2_3d::DataType;
  EXPECT_THROW(
    SE_2_3d(DataType::Random()*10.), manif::invalid_argument
  );
}

TEST(TEST_SE_2_3, TEST_SE_2_3_NORMALIZE)
{
  using DataType = SE_2_3d::DataType;
  DataType data = DataType::Random() * 100.;

  EXPECT_THROW(
    SE_2_3d a(data), manif::invalid_argument
  );

  Eigen::Map<SE_2_3d> map(data.data());
  map.normalize();

  EXPECT_NO_THROW(
    SE_2_3d b = map
  );
}

#endif

MANIF_TEST(SE_2_3d);

MANIF_TEST_MAP(SE_2_3d);

MANIF_TEST_JACOBIANS(SE_2_3d);

MANIF_RUN_ALL_TEST;
