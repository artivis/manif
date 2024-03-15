#include <gtest/gtest.h>

#include "manif/SGal3.h"
#include "../common_tester.h"

#include <Eigen/Geometry>
#include <iostream>
using namespace manif;

TEST(TEST_SGAL3, TEST_SGAL3_CONSTRUCTOR_DATATYPE) {
  SGal3d::DataType values; values << 0,0,0, 0,0,0,1, 0,0,0, 0;
  SGal3d sgal3(values);

  EXPECT_DOUBLE_EQ(0, sgal3.coeffs()(0));
  EXPECT_DOUBLE_EQ(0, sgal3.coeffs()(1));
  EXPECT_DOUBLE_EQ(0, sgal3.coeffs()(2));
  EXPECT_DOUBLE_EQ(0, sgal3.coeffs()(3));
  EXPECT_DOUBLE_EQ(0, sgal3.coeffs()(4));
  EXPECT_DOUBLE_EQ(0, sgal3.coeffs()(5));
  EXPECT_DOUBLE_EQ(1, sgal3.coeffs()(6));
  EXPECT_DOUBLE_EQ(0, sgal3.coeffs()(7));
  EXPECT_DOUBLE_EQ(0, sgal3.coeffs()(8));
  EXPECT_DOUBLE_EQ(0, sgal3.coeffs()(9));
  EXPECT_DOUBLE_EQ(0, sgal3.coeffs()(10));
}

TEST(TEST_SGAL3, TEST_SGAL3_CONSTRUCTOR_T_Q_V) {
  SGal3d sgal3(
    SGal3d::Translation(1,2,3),
    Eigen::Quaterniond::Identity(),
    SGal3d::LinearVelocity(4, 5, 6),
    1
  );

  EXPECT_DOUBLE_EQ(1, sgal3.coeffs()(0));
  EXPECT_DOUBLE_EQ(2, sgal3.coeffs()(1));
  EXPECT_DOUBLE_EQ(3, sgal3.coeffs()(2));
  EXPECT_DOUBLE_EQ(0, sgal3.coeffs()(3));
  EXPECT_DOUBLE_EQ(0, sgal3.coeffs()(4));
  EXPECT_DOUBLE_EQ(0, sgal3.coeffs()(5));
  EXPECT_DOUBLE_EQ(1, sgal3.coeffs()(6));
  EXPECT_DOUBLE_EQ(4, sgal3.coeffs()(7));
  EXPECT_DOUBLE_EQ(5, sgal3.coeffs()(8));
  EXPECT_DOUBLE_EQ(6, sgal3.coeffs()(9));
  EXPECT_DOUBLE_EQ(1, sgal3.coeffs()(10));

  EXPECT_EIGEN_NEAR(Eigen::Vector3d(1, 2, 3), sgal3.translation());
  EXPECT_EIGEN_NEAR(Eigen::Quaterniond::Identity().coeffs(), sgal3.quat().coeffs());
  EXPECT_EIGEN_NEAR(Eigen::Vector3d(4, 5, 6), sgal3.linearVelocity());
}

TEST(TEST_SGAL3, TEST_SGAL3_CONSTRUCTOR_T_AA_V) {
  SGal3d sgal3(
    SGal3d::Translation(1,2,3),
    Eigen::AngleAxis<double>(Eigen::Quaterniond::Identity()),
    SGal3d::LinearVelocity(1, 2, 3),
    1
  );

  EXPECT_DOUBLE_EQ(1, sgal3.coeffs()(0));
  EXPECT_DOUBLE_EQ(2, sgal3.coeffs()(1));
  EXPECT_DOUBLE_EQ(3, sgal3.coeffs()(2));
  EXPECT_DOUBLE_EQ(0, sgal3.coeffs()(3));
  EXPECT_DOUBLE_EQ(0, sgal3.coeffs()(4));
  EXPECT_DOUBLE_EQ(0, sgal3.coeffs()(5));
  EXPECT_DOUBLE_EQ(1, sgal3.coeffs()(6));
  EXPECT_DOUBLE_EQ(1, sgal3.coeffs()(7));
  EXPECT_DOUBLE_EQ(2, sgal3.coeffs()(8));
  EXPECT_DOUBLE_EQ(3, sgal3.coeffs()(9));
  EXPECT_DOUBLE_EQ(1, sgal3.coeffs()(10));
}

TEST(TEST_SGAL3, TEST_SGAL3_CONSTRUCTOR_ISOMETRY) {
  Eigen::Isometry3d h = Eigen::Translation3d(1,2,3) * Eigen::Quaterniond::Identity();
  SGal3d sgal3(h, Eigen::Vector3d(1, 2, 3), 1);

  Eigen::Matrix<double, 5, 5> hse23 = Eigen::Matrix<double, 5, 5>::Identity();
  hse23.topLeftCorner<4, 4>() = h.matrix();
  hse23.topRightCorner<3, 1>() = Eigen::Vector3d(1, 2, 3);
  hse23(3, 4) = 1;

  EXPECT_DOUBLE_EQ(1, sgal3.coeffs()(0));
  EXPECT_DOUBLE_EQ(2, sgal3.coeffs()(1));
  EXPECT_DOUBLE_EQ(3, sgal3.coeffs()(2));
  EXPECT_DOUBLE_EQ(0, sgal3.coeffs()(3));
  EXPECT_DOUBLE_EQ(0, sgal3.coeffs()(4));
  EXPECT_DOUBLE_EQ(0, sgal3.coeffs()(5));
  EXPECT_DOUBLE_EQ(1, sgal3.coeffs()(6));
  EXPECT_DOUBLE_EQ(1, sgal3.coeffs()(7));
  EXPECT_DOUBLE_EQ(2, sgal3.coeffs()(8));
  EXPECT_DOUBLE_EQ(3, sgal3.coeffs()(9));
  EXPECT_DOUBLE_EQ(1, sgal3.coeffs()(10));
  EXPECT_EIGEN_NEAR(hse23, sgal3.isometry());
}

TEST(TEST_SGAL3, TEST_SGAL3_CONSTRUCTOR_COPY) {
  SGal3d::DataType values; values << 3,2,1, 0,0,0,1, 1,2,3, 4;
  SGal3d o(values);
  SGal3d sgal3(o);

  EXPECT_DOUBLE_EQ(3, sgal3.coeffs()(0));
  EXPECT_DOUBLE_EQ(2, sgal3.coeffs()(1));
  EXPECT_DOUBLE_EQ(1, sgal3.coeffs()(2));
  EXPECT_DOUBLE_EQ(0, sgal3.coeffs()(3));
  EXPECT_DOUBLE_EQ(0, sgal3.coeffs()(4));
  EXPECT_DOUBLE_EQ(0, sgal3.coeffs()(5));
  EXPECT_DOUBLE_EQ(1, sgal3.coeffs()(6));
  EXPECT_DOUBLE_EQ(1, sgal3.coeffs()(7));
  EXPECT_DOUBLE_EQ(2, sgal3.coeffs()(8));
  EXPECT_DOUBLE_EQ(3, sgal3.coeffs()(9));
  EXPECT_DOUBLE_EQ(4, sgal3.coeffs()(10));
}

TEST(TEST_SGAL3, TEST_SGAL3_DATA) {
  SGal3d::DataType values; values << 3,2,1, 0,0,0,1, 1,2,3, 4;
  SGal3d sgal3(values);

  double * data_ptr = sgal3.data();

  ASSERT_NE(nullptr, data_ptr);

  EXPECT_DOUBLE_EQ(3, *data_ptr);
  ++data_ptr;
  EXPECT_DOUBLE_EQ(2, *data_ptr);
  ++data_ptr;
  EXPECT_DOUBLE_EQ(1, *data_ptr);
  ++data_ptr;
  EXPECT_DOUBLE_EQ(0, *data_ptr);
  ++data_ptr;
  EXPECT_DOUBLE_EQ(0, *data_ptr);
  ++data_ptr;
  EXPECT_DOUBLE_EQ(0, *data_ptr);
  ++data_ptr;
  EXPECT_DOUBLE_EQ(1, *data_ptr);
  ++data_ptr;
  EXPECT_DOUBLE_EQ(1, *data_ptr);
  ++data_ptr;
  EXPECT_DOUBLE_EQ(2, *data_ptr);
  ++data_ptr;
  EXPECT_DOUBLE_EQ(3, *data_ptr);
  ++data_ptr;
  EXPECT_DOUBLE_EQ(4, *data_ptr);
}

TEST(TEST_SGAL3, TEST_SGAL3_CAST) {
  SGal3d sgal3d(
    SGal3d::Translation(1,2,3),
    Eigen::Quaterniond::Identity(),
    SGal3d::LinearVelocity(1,2,3),
    1
  );

  EXPECT_DOUBLE_EQ(1, sgal3d.coeffs()(0));
  EXPECT_DOUBLE_EQ(2, sgal3d.coeffs()(1));
  EXPECT_DOUBLE_EQ(3, sgal3d.coeffs()(2));
  EXPECT_DOUBLE_EQ(0, sgal3d.coeffs()(3));
  EXPECT_DOUBLE_EQ(0, sgal3d.coeffs()(4));
  EXPECT_DOUBLE_EQ(0, sgal3d.coeffs()(5));
  EXPECT_DOUBLE_EQ(1, sgal3d.coeffs()(6));
  EXPECT_DOUBLE_EQ(1, sgal3d.coeffs()(7));
  EXPECT_DOUBLE_EQ(2, sgal3d.coeffs()(8));
  EXPECT_DOUBLE_EQ(3, sgal3d.coeffs()(9));
  EXPECT_DOUBLE_EQ(1, sgal3d.coeffs()(10));

  SGal3f sgal3f = sgal3d.cast<float>();

  EXPECT_DOUBLE_EQ(1, sgal3f.coeffs()(0));
  EXPECT_DOUBLE_EQ(2, sgal3f.coeffs()(1));
  EXPECT_DOUBLE_EQ(3, sgal3f.coeffs()(2));
  EXPECT_DOUBLE_EQ(0, sgal3f.coeffs()(3));
  EXPECT_DOUBLE_EQ(0, sgal3f.coeffs()(4));
  EXPECT_DOUBLE_EQ(0, sgal3f.coeffs()(5));
  EXPECT_DOUBLE_EQ(1, sgal3f.coeffs()(6));
  EXPECT_DOUBLE_EQ(1, sgal3f.coeffs()(7));
  EXPECT_DOUBLE_EQ(2, sgal3f.coeffs()(8));
  EXPECT_DOUBLE_EQ(3, sgal3f.coeffs()(9));
  EXPECT_DOUBLE_EQ(1, sgal3f.coeffs()(10));
}

TEST(TEST_SGAL3, TEST_SGAL3_IDENTITY) {
  SGal3d sgal3;

  sgal3.setIdentity();

  EXPECT_DOUBLE_EQ(0, sgal3.coeffs()(0));
  EXPECT_DOUBLE_EQ(0, sgal3.coeffs()(1));
  EXPECT_DOUBLE_EQ(0, sgal3.coeffs()(2));
  EXPECT_DOUBLE_EQ(0, sgal3.coeffs()(3));
  EXPECT_DOUBLE_EQ(0, sgal3.coeffs()(4));
  EXPECT_DOUBLE_EQ(0, sgal3.coeffs()(5));
  EXPECT_DOUBLE_EQ(1, sgal3.coeffs()(6));
  EXPECT_DOUBLE_EQ(0, sgal3.coeffs()(7));
  EXPECT_DOUBLE_EQ(0, sgal3.coeffs()(8));
  EXPECT_DOUBLE_EQ(0, sgal3.coeffs()(9));
  EXPECT_DOUBLE_EQ(0, sgal3.coeffs()(10));
}

TEST(TEST_SGAL3, TEST_SGAL3_IDENTITY2) {
  SGal3d sgal3 = SGal3d::Identity();

  EXPECT_DOUBLE_EQ(0, sgal3.coeffs()(0));
  EXPECT_DOUBLE_EQ(0, sgal3.coeffs()(1));
  EXPECT_DOUBLE_EQ(0, sgal3.coeffs()(2));
  EXPECT_DOUBLE_EQ(0, sgal3.coeffs()(3));
  EXPECT_DOUBLE_EQ(0, sgal3.coeffs()(4));
  EXPECT_DOUBLE_EQ(0, sgal3.coeffs()(5));
  EXPECT_DOUBLE_EQ(1, sgal3.coeffs()(6));
  EXPECT_DOUBLE_EQ(0, sgal3.coeffs()(7));
  EXPECT_DOUBLE_EQ(0, sgal3.coeffs()(8));
  EXPECT_DOUBLE_EQ(0, sgal3.coeffs()(9));
  EXPECT_DOUBLE_EQ(0, sgal3.coeffs()(10));
}

TEST(TEST_SGAL3, TEST_SGAL3_RANDOM) {
 SGal3d sgal3;

 sgal3.setRandom();

 const auto q_norm = sgal3.coeffs().segment<4>(3).norm();

 EXPECT_DOUBLE_EQ(1, q_norm);
}

TEST(TEST_SGAL3, TEST_SGAL3_RANDOM2) {
 const SGal3d sgal3 = SGal3d::Random();

 const auto q_norm = sgal3.coeffs().segment<4>(3).norm();

 EXPECT_DOUBLE_EQ(1, q_norm);
}

TEST(TEST_SGAL3, TEST_SGAL3_INVERSE) {
  SGal3d sgal3 = SGal3d::Identity();

  auto sgal3_inv = sgal3.inverse();

  EXPECT_DOUBLE_EQ(0, sgal3_inv.coeffs()(0));
  EXPECT_DOUBLE_EQ(0, sgal3_inv.coeffs()(1));
  EXPECT_DOUBLE_EQ(0, sgal3_inv.coeffs()(2));
  EXPECT_DOUBLE_EQ(0, sgal3_inv.coeffs()(3));
  EXPECT_DOUBLE_EQ(0, sgal3_inv.coeffs()(4));
  EXPECT_DOUBLE_EQ(0, sgal3_inv.coeffs()(5));
  EXPECT_DOUBLE_EQ(1, sgal3_inv.coeffs()(6));
  EXPECT_DOUBLE_EQ(0, sgal3_inv.coeffs()(7));
  EXPECT_DOUBLE_EQ(0, sgal3_inv.coeffs()(8));
  EXPECT_DOUBLE_EQ(0, sgal3_inv.coeffs()(9));
  EXPECT_DOUBLE_EQ(0, sgal3_inv.coeffs()(10));

  sgal3 = SGal3d::Random();

  EXPECT_MANIF_NEAR(SGal3d::Identity(), sgal3.inverse()*sgal3);
  EXPECT_MANIF_NEAR(SGal3d::Identity(), sgal3*sgal3.inverse());
}

TEST(TEST_SGAL3, TEST_SGAL3_COMPOSE_IDENTITY) {
  SGal3d sgal3a = SGal3d::Identity();
  SGal3d sgal3b = SGal3d::Identity();

  auto sgal3c = sgal3a.compose(sgal3b);

  EXPECT_DOUBLE_EQ(0, sgal3c.coeffs()(0));
  EXPECT_DOUBLE_EQ(0, sgal3c.coeffs()(1));
  EXPECT_DOUBLE_EQ(0, sgal3c.coeffs()(2));
  EXPECT_DOUBLE_EQ(0, sgal3c.coeffs()(3));
  EXPECT_DOUBLE_EQ(0, sgal3c.coeffs()(4));
  EXPECT_DOUBLE_EQ(0, sgal3c.coeffs()(5));
  EXPECT_DOUBLE_EQ(1, sgal3c.coeffs()(6));
  EXPECT_DOUBLE_EQ(0, sgal3c.coeffs()(7));
  EXPECT_DOUBLE_EQ(0, sgal3c.coeffs()(8));
  EXPECT_DOUBLE_EQ(0, sgal3c.coeffs()(9));
  EXPECT_DOUBLE_EQ(0, sgal3c.coeffs()(10));

  sgal3a = SGal3d::Random();
  sgal3c = sgal3a.compose(sgal3b);

  EXPECT_MANIF_NEAR(sgal3a, sgal3c);
}

TEST(TEST_SGAL3, TEST_SGAL3_LOG_EXP) {
  SGal3d sgal3 = SGal3d::Random();

  EXPECT_MANIF_NEAR(sgal3, sgal3.log().exp());
}

// TEST(TEST_SGAL3, TEST_SGAL3_ACT)
// {
//   SGal3d sgal3 = SGal3d::Identity();

//   auto transformed_point = sgal3.act(Eigen::Vector3d(1,1,1));

//   EXPECT_NEAR(+1, transformed_point.x(), 1e-15);
//   EXPECT_NEAR(+1, transformed_point.y(), 1e-15);
//   EXPECT_NEAR(+1, transformed_point.z(), 1e-15);

//   sgal3 = SGal3d(1,1,1,MANIF_PI,MANIF_PI_2,MANIF_PI/4.,0,0,0);

//   transformed_point = sgal3.act(Eigen::Vector3d(1,1,1));

//   EXPECT_NEAR( 1, transformed_point.x(), 1e-15);
//   EXPECT_NEAR(-0.414213562373, transformed_point.y(), 1e-12);
//   EXPECT_NEAR( 0, transformed_point.z(), 1e-15);

//   // to demonstrate that the velocity terms do not affect the rigid motion action
//   sgal3 = SGal3d(-1,-1,-1,MANIF_PI/4,-MANIF_PI_2,-MANIF_PI,0,0,1);

//   transformed_point = sgal3.act(Eigen::Vector3d(1,1,1));

//   EXPECT_NEAR( 0.414213562373, transformed_point.x(), 1e-12);
//   EXPECT_NEAR(-1, transformed_point.y(), 1e-15);
//   EXPECT_NEAR( 0, transformed_point.z(), 1e-15);
// }

TEST(TEST_SGAL3, TEST_SGAL3TAN_LINANGVELACC) {
  SGal3Tangentd::DataType data;
  data << 1,2,3, 4,5,6, 7,8,9, 10;
  SGal3Tangentd se23tan(data);

  EXPECT_EIGEN_NEAR(Eigen::Vector3d(1, 2, 3), se23tan.lin());
  EXPECT_EIGEN_NEAR(Eigen::Vector3d(4, 5, 6), se23tan.lin2());
  EXPECT_EIGEN_NEAR(Eigen::Vector3d(7, 8, 9), se23tan.ang());
  EXPECT_DOUBLE_EQ(10, se23tan.t());
}

#ifndef MANIF_NO_DEBUG

TEST(TEST_SGAL3, TEST_SGAL3_CONSTRUCTOR_NOT_NORMALIZED_ARGS) {
  SGal3d::DataType values; values << 0,0,0, 1,1,1,1, 0,0,0, 0;

  EXPECT_THROW(SGal3d sgal3(values), manif::invalid_argument);

  try {
    SGal3d sgal3(values);
  } catch (manif::invalid_argument& e) {
    EXPECT_FALSE(std::string(e.what()).empty());
  }
}

TEST(TEST_SGAL3, TEST_SGAL3_CONSTRUCTOR_UNNORMALIZED) {
  EXPECT_THROW(SGal3d(SGal3d::DataType::Random()*10.), manif::invalid_argument);
}

TEST(TEST_SGAL3, TEST_SGAL3_NORMALIZE) {
  using DataType = SGal3d::DataType;
  DataType data = DataType::Random() * 100.;

  EXPECT_THROW(SGal3d a(data), manif::invalid_argument);

  Eigen::Map<SGal3d> map(data.data());
  map.normalize();

  EXPECT_NO_THROW(SGal3d b = map);
}

#endif

MANIF_TEST(SGal3d);

MANIF_TEST_MAP(SGal3d);

MANIF_TEST_JACOBIANS(SGal3d);

MANIF_RUN_ALL_TEST;
