#include <gtest/gtest.h>

#include "../test_utils.h"

#include "manif/SO2.h"

using namespace manif;

TEST(TEST_SO2, TEST_SO2_BASE_TEMPLATE_TANGENT_DATA)
{
  /// @todo without specifying const
  /// it calls non-const data()
  const SO2Tangentd so2tan(M_PI);

  const SO2Tangentd::DataType& data_ref =
      callCoeffs(so2tan);

  EXPECT_DOUBLE_EQ(M_PI, data_ref(0));
}

TEST(TEST_SO2, TEST_SO2_BASE_TEMPLATE_TANGENT_ZERO)
{
  SO2Tangentd so2tan;

  callZero(so2tan);

  EXPECT_DOUBLE_EQ(0, so2tan.angle());
}

//TEST(TEST_SO2, TEST_SO2_BASE_TEMPLATE_RANDOM)
//{
//  SO2Tangentd so2tan;

//  callRandom(so2tan).random();

//  EXPECT_DOUBLE_EQ(0, so2tan.angle());
//}

TEST(TEST_SO2, TEST_SO2_BASE_TEMPLATE_TANGENT_RETRACT)
{
  SO2Tangentd so2_tan(M_PI);

  EXPECT_DOUBLE_EQ(M_PI, so2_tan.angle());

  auto so2_retract = callRetract(so2_tan);

  EXPECT_DOUBLE_EQ(std::cos(M_PI), so2_retract.real());
  EXPECT_DOUBLE_EQ(std::sin(M_PI), so2_retract.imag());
  EXPECT_DOUBLE_EQ(M_PI, so2_retract.angle());
}

/// with Jacs

TEST(TEST_SO2, TEST_SO2_BASE_TEMPLATE_TANGENT_RETRACT_JAC)
{
  SO2Tangentd so2_tan(M_PI);

  EXPECT_DOUBLE_EQ(M_PI, so2_tan.angle());

  SO2d so2_retract;
  SO2d::Jacobian J_ret;

  callRetractWithJac(so2_tan, so2_retract, J_ret);

  EXPECT_DOUBLE_EQ(std::cos(M_PI), so2_retract.real());
  EXPECT_DOUBLE_EQ(std::sin(M_PI), so2_retract.imag());
  EXPECT_DOUBLE_EQ(M_PI, so2_retract.angle());

  /// @todo check this J
  EXPECT_EQ(1, J_ret.rows());
  EXPECT_EQ(1, J_ret.cols());
  EXPECT_DOUBLE_EQ(1, J_ret(0));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
