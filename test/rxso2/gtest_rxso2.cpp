#include <gtest/gtest.h>

#include "manif/RxSO2.h"

#include "../common_tester.h"

using namespace manif;

TEST(TEST_RxSO2, TEST_RxSO2_CONSTRUCTOR_DATATYPE)
{
  RxSO2d rxso3((RxSO2d::DataType() << 0,1,1).finished());

  EXPECT_DOUBLE_EQ(0, rxso3.real());
  EXPECT_DOUBLE_EQ(1, rxso3.imag());
  EXPECT_DOUBLE_EQ(1, rxso3.scale());
}

TEST(TEST_RxSO2, TEST_RxSO2_CONSTRUCTOR_QUAT_COEFFS)
{
  RxSO2d rxso3(0,1,1);

  EXPECT_DOUBLE_EQ(0, rxso3.real());
  EXPECT_DOUBLE_EQ(1, rxso3.imag());
  EXPECT_DOUBLE_EQ(1, rxso3.scale());
}

#ifndef MANIF_NO_DEBUG

TEST(TEST_RxSO2, TEST_RxSO2_CONSTRUCTOR_NOT_NORMALIZED_ARGS)
{
  EXPECT_THROW(
    RxSO2d so3(RxSO2d(0, 0, 1)),
    manif::invalid_argument
  );

  EXPECT_THROW(
    RxSO2d so3((RxSO2d::DataType() << 0, 0, 1).finished()),
    manif::invalid_argument
  );

  EXPECT_THROW(
    RxSO2d so3((RxSO2d::DataType() << 0, 1, 0).finished()),
    manif::invalid_argument
  );

  try {
    RxSO2d so3((RxSO2d::DataType() << 0, 0, 0).finished());
  } catch (manif::invalid_argument& e) {
    EXPECT_FALSE(std::string(e.what()).empty());
  }
}

TEST(TEST_RxSO2, TEST_RxSO2_CONSTRUCTOR_UNNORMALIZED)
{
  using DataType = typename RxSO2d::DataType;
  EXPECT_THROW(
    RxSO2d(DataType::Random()*10.), manif::invalid_argument
  );
}

TEST(TEST_RxSO2, TEST_RxSO2_NORMALIZE)
{
  using DataType = RxSO2d::DataType;
  DataType data = DataType::Random() * 100.;

  data(2) = 1; // scale ain't normalized

  EXPECT_THROW(
    RxSO2d a(data), manif::invalid_argument
  );

  Eigen::Map<RxSO2d> map(data.data());
  map.normalize();

  EXPECT_NO_THROW(
    RxSO2d b = map
  );
}

#endif

MANIF_TEST(RxSO2f);

MANIF_TEST_MAP(RxSO2f);

MANIF_TEST_JACOBIANS(RxSO2f);

MANIF_TEST(RxSO2d);

MANIF_TEST_MAP(RxSO2d);

MANIF_TEST_JACOBIANS(RxSO2d);

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
