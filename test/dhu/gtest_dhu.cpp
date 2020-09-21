#include <gtest/gtest.h>

#include "manif/DHu.h"

#include "../common_tester.h"

using namespace manif;

TEST(TEST_DHU, TEST_DHU_CONSTRUCTOR_DATATYPE)
{
  DHud dhu((DHud::DataType() << 0,0,0,1,0,0,0,0).finished());

  EXPECT_DOUBLE_EQ(0, dhu.coeffs()(0));
  EXPECT_DOUBLE_EQ(0, dhu.coeffs()(1));
  EXPECT_DOUBLE_EQ(0, dhu.coeffs()(2));
  EXPECT_DOUBLE_EQ(1, dhu.coeffs()(3));
  EXPECT_DOUBLE_EQ(0, dhu.coeffs()(4));
  EXPECT_DOUBLE_EQ(0, dhu.coeffs()(5));
  EXPECT_DOUBLE_EQ(0, dhu.coeffs()(6));
  EXPECT_DOUBLE_EQ(0, dhu.coeffs()(7));
}

TEST(TEST_DHU, TEST_DHU_CONSTRUCTOR_QUAT)
{
  Eigen::Quaterniond real(1,0,0,0), dual(0,0,0,0);
  DHud dhu(real, dual);

  EXPECT_DOUBLE_EQ(0, dhu.coeffs()(0));
  EXPECT_DOUBLE_EQ(0, dhu.coeffs()(1));
  EXPECT_DOUBLE_EQ(0, dhu.coeffs()(2));
  EXPECT_DOUBLE_EQ(1, dhu.coeffs()(3));
  EXPECT_DOUBLE_EQ(0, dhu.coeffs()(4));
  EXPECT_DOUBLE_EQ(0, dhu.coeffs()(5));
  EXPECT_DOUBLE_EQ(0, dhu.coeffs()(6));
  EXPECT_DOUBLE_EQ(0, dhu.coeffs()(7));

  EXPECT_EIGEN_NEAR(dhu.real().coeffs(), real.coeffs());
  EXPECT_EIGEN_NEAR(dhu.dual().coeffs(), dual.coeffs());
}

// TEST(TEST_DHU, TEST_DHU_IDENTITY)
// {
//   DHud dhu;

//   dhu.setIdentity();

//   EXPECT_DOUBLE_EQ(0, dhu.coeffs()(0));
//   EXPECT_DOUBLE_EQ(0, dhu.coeffs()(1));
//   EXPECT_DOUBLE_EQ(0, dhu.coeffs()(2));
//   EXPECT_DOUBLE_EQ(1, dhu.coeffs()(3));
//   EXPECT_DOUBLE_EQ(0, dhu.coeffs()(4));
//   EXPECT_DOUBLE_EQ(0, dhu.coeffs()(5));
//   EXPECT_DOUBLE_EQ(0, dhu.coeffs()(6));
//   EXPECT_DOUBLE_EQ(0, dhu.coeffs()(7));
// }

// TEST(TEST_DHU, TEST_DHU_IDENTITY2)
// {
//   const DHud dhu = DHud::Identity();

//   EXPECT_DOUBLE_EQ(0, dhu.coeffs()(0));
//   EXPECT_DOUBLE_EQ(0, dhu.coeffs()(1));
//   EXPECT_DOUBLE_EQ(0, dhu.coeffs()(2));
//   EXPECT_DOUBLE_EQ(1, dhu.coeffs()(3));
//   EXPECT_DOUBLE_EQ(0, dhu.coeffs()(4));
//   EXPECT_DOUBLE_EQ(0, dhu.coeffs()(5));
//   EXPECT_DOUBLE_EQ(0, dhu.coeffs()(6));
//   EXPECT_DOUBLE_EQ(0, dhu.coeffs()(7));
// }

TEST(TEST_DHU, TEST_DHU_INVERSE)
{
  // inverse of identity is identity
  // DHud dhu = DHud::Identity();
  DHud dhu((DHud::DataType() << 0,0,0,1,0,0,0,0).finished());

  auto dhu_inv = dhu.inverse();

  EXPECT_DOUBLE_EQ(0, dhu_inv.coeffs()(0));
  EXPECT_DOUBLE_EQ(0, dhu_inv.coeffs()(1));
  EXPECT_DOUBLE_EQ(0, dhu_inv.coeffs()(2));
  EXPECT_DOUBLE_EQ(1, dhu_inv.coeffs()(3));
  EXPECT_DOUBLE_EQ(0, dhu_inv.coeffs()(4));
  EXPECT_DOUBLE_EQ(0, dhu_inv.coeffs()(5));
  EXPECT_DOUBLE_EQ(0, dhu_inv.coeffs()(6));
  EXPECT_DOUBLE_EQ(0, dhu_inv.coeffs()(7));

  // inverse of random in quaternion form is conjugate
  EXPECT_NO_THROW(
    dhu = DHud::Random();
  );

  EXPECT_NO_THROW(
    dhu_inv = dhu.inverse();
  );

  DHud I((DHud::DataType() << 0,0,0,1,0,0,0,0).finished());

  EXPECT_MANIF_NEAR(I, dhu*dhu_inv);
  EXPECT_MANIF_NEAR(I, dhu_inv*dhu);
}

#ifndef MANIF_NO_DEBUG

TEST(TEST_DHU, TEST_DHU_CONSTRUCTOR_NOT_NORMALIZED_ARGS)
{
  EXPECT_THROW(
    DHud dhu((DHud::DataType() << 1,1,1,1,0,0,0,0).finished()),
    manif::invalid_argument
  );

  EXPECT_THROW(
    DHud dhu((DHud::DataType() << 0,0,0,1,1,1,1,1).finished()),
    manif::invalid_argument
  );

  try {
    DHud dhu((DHud::DataType() << 1,1,1,1,0,0,0,0).finished());
  } catch (manif::invalid_argument& e) {
    EXPECT_FALSE(std::string(e.what()).empty());
  }
}

TEST(TEST_DHU, TEST_DHU_CONSTRUCTOR_UNNORMALIZED)
{
  using DataType = typename DHud::DataType;
  EXPECT_THROW(
    DHud(DataType::Random()*10.), manif::invalid_argument
  );
}

TEST(TEST_DHU, TEST_DHU_NORMALIZE)
{
  using DataType = DHud::DataType;
  DataType data = DataType::Random() * 100.;

  EXPECT_THROW(
    DHud a(data), manif::invalid_argument
  );

  Eigen::Map<DHud> map(data.data());
  map.normalize();

  EXPECT_NO_THROW(
    DHud b = map
  );

  try {
    DHud b = map;
  } catch (manif::invalid_argument& e) {
    EXPECT_FALSE(true) << e.what();
  }
}

#endif

// MANIF_TEST(DHud);

// MANIF_TEST_MAP(DHud);

// MANIF_TEST_JACOBIANS(DHud);

int main(int argc, char** argv)
{
  std::srand((unsigned int) time(0));
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
