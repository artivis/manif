#include <gtest/gtest.h>

#include "manif/Sim2.h"

#include "../common_tester.h"

using namespace manif;

TEST(TEST_SIM2, TEST_SIM2_CONSTRUCTOR_DATATYPE)
{
  Sim2d sim2((Sim2d::DataType()<<0,0,1,0,1).finished());

  EXPECT_DOUBLE_EQ(0, sim2.x());
  EXPECT_DOUBLE_EQ(0, sim2.y());
  EXPECT_DOUBLE_EQ(0, sim2.angle());
  EXPECT_DOUBLE_EQ(1, sim2.scale());
}

TEST(TEST_SIM2, TEST_SIM2_CONSTRUCTOR_X_Y_REAL_IMAG_SCALE)
{
  Sim2d sim2(4, 2, 1, 0, 2);

  EXPECT_DOUBLE_EQ(4, sim2.x());
  EXPECT_DOUBLE_EQ(2, sim2.y());
  EXPECT_DOUBLE_EQ(1, sim2.real());
  EXPECT_DOUBLE_EQ(0, sim2.imag());
  EXPECT_DOUBLE_EQ(0, sim2.angle());
  EXPECT_DOUBLE_EQ(2, sim2.scale());
}

TEST(TEST_SIM2, TEST_SIM2_CONSTRUCTOR_X_Y_THETA_SCALE)
{
  Sim2d sim2(4, 2, 0, 1);

  EXPECT_DOUBLE_EQ(4, sim2.x());
  EXPECT_DOUBLE_EQ(2, sim2.y());
  EXPECT_DOUBLE_EQ(0, sim2.angle());
  EXPECT_DOUBLE_EQ(1, sim2.scale());
}

TEST(TEST_SIM2, TEST_SIM2_CONSTRUCTOR_X_Y_C)
{
  Sim2d sim2(Eigen::Vector2d(4., 2.), std::complex<double>(1, 0), 1.);

  EXPECT_DOUBLE_EQ(4, sim2.x());
  EXPECT_DOUBLE_EQ(2, sim2.y());
  EXPECT_DOUBLE_EQ(1, sim2.real());
  EXPECT_DOUBLE_EQ(0, sim2.imag());
  EXPECT_DOUBLE_EQ(0, sim2.angle());
  EXPECT_DOUBLE_EQ(1, sim2.scale());
}

#ifndef MANIF_NO_DEBUG

TEST(TEST_SIM2, TEST_SIM2_CONSTRUCTOR_NOT_NORMALIZED_ARGS)
{
  EXPECT_THROW(
    Sim2d sim2(Sim2d(4, 2, 1, 1, 1)),
    manif::invalid_argument
  );

  EXPECT_THROW(
    Sim2d sim2((Sim2d::DataType()<<4, 2, 1, 1, 1).finished()),
    manif::invalid_argument
  );

  EXPECT_THROW(
    Sim2d sim2((Sim2d::DataType()<<4, 2, 1, 0, 0).finished()),
    manif::invalid_argument
  );

  try {
    Sim2d sim2((Sim2d::DataType()<<4, 2, 1, 1, 1).finished());
  } catch (manif::invalid_argument& e) {
    EXPECT_FALSE(std::string(e.what()).empty());
  }
}

TEST(TEST_SIM2, TEST_SIM2_CONSTRUCTOR_UNNORMALIZED)
{
  using DataType = typename Sim2d::DataType;
  EXPECT_THROW(
    Sim2d(DataType::Random()*10.), manif::invalid_argument
  );
}

TEST(TEST_SIM2, TEST_SIM2_NORMALIZE)
{
  using DataType = Sim2d::DataType;
  DataType data = DataType::Random() * 100.;
  data(4) = 1.;

  EXPECT_THROW(
    Sim2d a(data), manif::invalid_argument
  );

  Eigen::Map<Sim2d> map(data.data());
  map.normalize();

  EXPECT_NO_THROW(
    Sim2d b = map
  );
}

#endif

// MANIF_TEST(Sim2f);

// MANIF_TEST_MAP(Sim2f);

// MANIF_TEST_JACOBIANS(Sim2f);

MANIF_TEST(Sim2d);

// MANIF_TEST_MAP(Sim2d);

// MANIF_TEST_JACOBIANS(Sim2d);

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
