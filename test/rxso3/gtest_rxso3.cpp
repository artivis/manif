#include <gtest/gtest.h>

#include "manif/RxSO3.h"

#include "../common_tester.h"

using namespace manif;

TEST(TEST_RxSO3, TEST_RxSO3_CONSTRUCTOR_DATATYPE)
{
  RxSO3d rxso3((RxSO3d::DataType() << 0,0,0,1,1).finished());

  EXPECT_DOUBLE_EQ(0, rxso3.x());
  EXPECT_DOUBLE_EQ(0, rxso3.y());
  EXPECT_DOUBLE_EQ(0, rxso3.z());
  EXPECT_DOUBLE_EQ(1, rxso3.w());
  EXPECT_DOUBLE_EQ(1, rxso3.scale());
}

TEST(TEST_RxSO3, TEST_RxSO3_CONSTRUCTOR_QUAT)
{
  RxSO3d rxso3(Eigen::Quaterniond(1,0,0,0),1);

  EXPECT_DOUBLE_EQ(0, rxso3.x());
  EXPECT_DOUBLE_EQ(0, rxso3.y());
  EXPECT_DOUBLE_EQ(0, rxso3.z());
  EXPECT_DOUBLE_EQ(1, rxso3.w());
  EXPECT_DOUBLE_EQ(1, rxso3.scale());
}

TEST(TEST_RxSO3, TEST_RxSO3_CONSTRUCTOR_QUAT_COEFFS)
{
  RxSO3d rxso3(0,0,0,1,1);

  EXPECT_DOUBLE_EQ(0, rxso3.x());
  EXPECT_DOUBLE_EQ(0, rxso3.y());
  EXPECT_DOUBLE_EQ(0, rxso3.z());
  EXPECT_DOUBLE_EQ(1, rxso3.w());
  EXPECT_DOUBLE_EQ(1, rxso3.scale());
}

TEST(TEST_RxSO3, TEST_RxSO3_CONSTRUCTOR_ANGLE_AXIS)
{
  RxSO3d rxso3(Eigen::AngleAxis<double>(0, Eigen::Vector3d::UnitX()),1);

  EXPECT_DOUBLE_EQ(0, rxso3.x());
  EXPECT_DOUBLE_EQ(0, rxso3.y());
  EXPECT_DOUBLE_EQ(0, rxso3.z());
  EXPECT_DOUBLE_EQ(1, rxso3.w());
  EXPECT_DOUBLE_EQ(1, rxso3.scale());
}

TEST(TEST_RxSO3, TEST_RxSO3_CONSTRUCTOR_ROLL_PITCH_YAW)
{
  RxSO3d rxso3(0,0,0,1);

  EXPECT_DOUBLE_EQ(0, rxso3.x());
  EXPECT_DOUBLE_EQ(0, rxso3.y());
  EXPECT_DOUBLE_EQ(0, rxso3.z());
  EXPECT_DOUBLE_EQ(1, rxso3.w());
  EXPECT_DOUBLE_EQ(1, rxso3.scale());
}

#ifndef MANIF_NO_DEBUG

TEST(TEST_RxSO3, TEST_RxSO3_CONSTRUCTOR_NOT_NORMALIZED_ARGS)
{
  EXPECT_THROW(
    RxSO3d so3(RxSO3d(1, 1, 1, 1, 0)),
    manif::invalid_argument
  );

  EXPECT_THROW(
    RxSO3d so3((RxSO3d::DataType() << 1, 1, 1, 1, 0).finished()),
    manif::invalid_argument
  );

  EXPECT_THROW(
    RxSO3d so3((RxSO3d::DataType() << 0, 0, 0, 1, 0).finished()),
    manif::invalid_argument
  );

  try {
    RxSO3d so3((RxSO3d::DataType() << 1, 1, 1, 1, 0).finished());
  } catch (manif::invalid_argument& e) {
    EXPECT_FALSE(std::string(e.what()).empty());
  }
}

TEST(TEST_RxSO3, TEST_RxSO3_CONSTRUCTOR_UNNORMALIZED)
{
  using DataType = typename RxSO3d::DataType;
  EXPECT_THROW(
    RxSO3d(DataType::Random()*10.), manif::invalid_argument
  );
}

TEST(TEST_RxSO3, TEST_RxSO3_NORMALIZE)
{
  using DataType = RxSO3d::DataType;
  DataType data = DataType::Random() * 100.;

  data(4) = 1; // scale ain't normalized

  EXPECT_THROW(
    RxSO3d a(data), manif::invalid_argument
  );

  Eigen::Map<RxSO3d> map(data.data());
  map.normalize();

  EXPECT_NO_THROW(
    RxSO3d b = map
  );
}

#endif

MANIF_TEST(RxSO3f);

MANIF_TEST_MAP(RxSO3f);

MANIF_TEST_JACOBIANS(RxSO3f);

MANIF_TEST(RxSO3d);

MANIF_TEST_MAP(RxSO3d);

MANIF_TEST_JACOBIANS(RxSO3d);

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
