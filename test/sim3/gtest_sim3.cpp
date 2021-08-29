#include <gtest/gtest.h>

#include "manif/Sim3.h"

#include "../common_tester.h"

using namespace manif;

TEST(TEST_Sim3, TEST_Sim3_CONSTRUCTOR_DATATYPE)
{
  Sim3d sim3((Sim3d::DataType() << 1,2,3,0,0,0,1,1).finished());

  EXPECT_DOUBLE_EQ(1, sim3.coeffs()(0));
  EXPECT_DOUBLE_EQ(2, sim3.coeffs()(1));
  EXPECT_DOUBLE_EQ(3, sim3.coeffs()(2));
  EXPECT_DOUBLE_EQ(0, sim3.coeffs()(3));
  EXPECT_DOUBLE_EQ(0, sim3.coeffs()(4));
  EXPECT_DOUBLE_EQ(0, sim3.coeffs()(5));
  EXPECT_DOUBLE_EQ(1, sim3.coeffs()(6));
  EXPECT_DOUBLE_EQ(1, sim3.coeffs()(7));
}

TEST(TEST_Sim3, TEST_Sim3_CONSTRUCTOR_QUAT)
{
  Sim3d sim3(
    Sim3d::Translation(1,2,3), Eigen::Quaterniond(1,0,0,0), 1
  );

  EXPECT_DOUBLE_EQ(1, sim3.coeffs()(0));
  EXPECT_DOUBLE_EQ(2, sim3.coeffs()(1));
  EXPECT_DOUBLE_EQ(3, sim3.coeffs()(2));
  EXPECT_DOUBLE_EQ(0, sim3.coeffs()(3));
  EXPECT_DOUBLE_EQ(0, sim3.coeffs()(4));
  EXPECT_DOUBLE_EQ(0, sim3.coeffs()(5));
  EXPECT_DOUBLE_EQ(1, sim3.coeffs()(6));
  EXPECT_DOUBLE_EQ(1, sim3.coeffs()(7));
}

TEST(TEST_Sim3, TEST_Sim3_CONSTRUCTOR_ANGLE_AXIS)
{
  Sim3d sim3(
    Sim3d::Translation(1,2,3),
    Eigen::AngleAxis<double>(0, Eigen::Vector3d::UnitX()),
    1
  );

  EXPECT_DOUBLE_EQ(1, sim3.coeffs()(0));
  EXPECT_DOUBLE_EQ(2, sim3.coeffs()(1));
  EXPECT_DOUBLE_EQ(3, sim3.coeffs()(2));
  EXPECT_DOUBLE_EQ(0, sim3.coeffs()(3));
  EXPECT_DOUBLE_EQ(0, sim3.coeffs()(4));
  EXPECT_DOUBLE_EQ(0, sim3.coeffs()(5));
  EXPECT_DOUBLE_EQ(1, sim3.coeffs()(6));
  EXPECT_DOUBLE_EQ(1, sim3.coeffs()(7));
}

TEST(TEST_Sim3, TEST_Sim3_CONSTRUCTOR_ROLL_PITCH_YAW)
{
  Eigen::Isometry3d h = Eigen::Translation3d(1,2,3) * Eigen::Quaterniond::Identity();
  Sim3d sim3(h, 1);

  EXPECT_DOUBLE_EQ(1, sim3.coeffs()(0));
  EXPECT_DOUBLE_EQ(2, sim3.coeffs()(1));
  EXPECT_DOUBLE_EQ(3, sim3.coeffs()(2));
  EXPECT_DOUBLE_EQ(0, sim3.coeffs()(3));
  EXPECT_DOUBLE_EQ(0, sim3.coeffs()(4));
  EXPECT_DOUBLE_EQ(0, sim3.coeffs()(5));
  EXPECT_DOUBLE_EQ(1, sim3.coeffs()(6));
  EXPECT_DOUBLE_EQ(1, sim3.coeffs()(7));
}

#ifndef MANIF_NO_DEBUG

TEST(TEST_Sim3, TEST_Sim3_CONSTRUCTOR_NOT_NORMALIZED_ARGS)
{
  // EXPECT_THROW(
  //   Sim3d sim3(Sim3d(1, 2, 3, 1, 1, 1, 1, 0)),
  //   manif::invalid_argument
  // );

  EXPECT_THROW(
    Sim3d sim3((Sim3d::DataType() << 1, 2, 3, 1, 1, 1, 1, 0).finished()),
    manif::invalid_argument
  );

  EXPECT_THROW(
    Sim3d sim3((Sim3d::DataType() << 1, 2, 3, 0, 0, 0, 1, 0).finished()),
    manif::invalid_argument
  );

  try {
    Sim3d sim3((Sim3d::DataType() << 1, 2, 3, 1, 1, 1, 1, 0).finished());
  } catch (manif::invalid_argument& e) {
    EXPECT_FALSE(std::string(e.what()).empty());
  }
}

TEST(TEST_Sim3, TEST_Sim3_CONSTRUCTOR_UNNORMALIZED)
{
  using DataType = typename Sim3d::DataType;
  EXPECT_THROW(
    Sim3d(DataType::Random()*10.), manif::invalid_argument
  );
}

TEST(TEST_Sim3, TEST_Sim3_NORMALIZE)
{
  using DataType = Sim3d::DataType;
  DataType data = DataType::Random() * 100.;

  data(7) = 1; // scale ain't normalized

  EXPECT_THROW(
    Sim3d a(data), manif::invalid_argument
  );

  Eigen::Map<Sim3d> map(data.data());
  map.normalize();

  EXPECT_NO_THROW(
    Sim3d b = map
  );
}

#endif

// MANIF_TEST(Sim3f);

// MANIF_TEST_MAP(Sim3f);

// MANIF_TEST_JACOBIANS(Sim3f);

MANIF_TEST(Sim3d);

// MANIF_TEST_MAP(Sim3d);

// MANIF_TEST_JACOBIANS(Sim3d);

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
