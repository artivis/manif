#include <gtest/gtest.h>

#include "manif/Rn.h"

#include "../common_tester.h"

#include <Eigen/StdVector>

using namespace manif;

// specialize std::vector for 'fixed-size vectorizable' Eigen object
// that are multiple of 32 bytes
// @todo: investigate why only this alignment is troublesome
// especially, SO3 wasn't an issue despite being Eigen::Vector4d too...
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(R4d)

#if defined(MANIF_COVERAGE_ENABLED) || defined(MANIF_ARCH_32)

MANIF_TEST(R4d);
MANIF_TEST_JACOBIANS(R4d);

#else

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(R8d)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(R8f)

TEST(TEST_RN, TEST_RN_VEC_ASSIGN_OP)
{
  {
    R1d::DataType data = R1d::DataType::Random();

    R1d r1;
    r1 = data;

    EXPECT_EIGEN_NEAR(data, r1.coeffs());
  }

  {
    R2d::DataType data = R2d::DataType::Random();

    R2d r2;
    r2 = data;

    EXPECT_EIGEN_NEAR(data, r2.coeffs());
  }

  {
    R3d::DataType data = R3d::DataType::Random();

    R3d r3;
    r3 = data;

    EXPECT_EIGEN_NEAR(data, r3.coeffs());
  }

  {
    R4d::DataType data = R4d::DataType::Random();

    R4d r4;
    r4 = data;

    EXPECT_EIGEN_NEAR(data, r4.coeffs());
  }

  {
    R5d::DataType data = R5d::DataType::Random();

    R5d r5;
    r5 = data;

    EXPECT_EIGEN_NEAR(data, r5.coeffs());
  }

  {
    R6d::DataType data = R6d::DataType::Random();

    R6d r6;
    r6 = data;

    EXPECT_EIGEN_NEAR(data, r6.coeffs());
  }

  {
    R7d::DataType data = R7d::DataType::Random();

    R7d r7;
    r7 = data;

    EXPECT_EIGEN_NEAR(data, r7.coeffs());
  }

  {
    R8d::DataType data = R8d::DataType::Random();

    R8d r8;
    r8 = data;

    EXPECT_EIGEN_NEAR(data, r8.coeffs());
  }

  {
    R9d::DataType data = R9d::DataType::Random();

    R9d r9;
    r9 = data;

    EXPECT_EIGEN_NEAR(data, r9.coeffs());
  }
}

// This is a little too heavy for coverage and not relevant...
// The same applies to 32-bit platforms.

MANIF_TEST(R1f);
MANIF_TEST(R2f);
MANIF_TEST(R3f);
MANIF_TEST(R4f);
MANIF_TEST(R5f);
MANIF_TEST(R6f);
MANIF_TEST(R7f);
MANIF_TEST(R8f);
MANIF_TEST(R9f);

MANIF_TEST_MAP(R1f);
MANIF_TEST_MAP(R2f);
MANIF_TEST_MAP(R3f);
MANIF_TEST_MAP(R4f);
MANIF_TEST_MAP(R5f);
MANIF_TEST_MAP(R6f);
MANIF_TEST_MAP(R7f);
MANIF_TEST_MAP(R8f);
MANIF_TEST_MAP(R9f);

MANIF_TEST_JACOBIANS(R1f);
MANIF_TEST_JACOBIANS(R2f);
MANIF_TEST_JACOBIANS(R3f);
MANIF_TEST_JACOBIANS(R4f);
MANIF_TEST_JACOBIANS(R5f);
MANIF_TEST_JACOBIANS(R6f);
MANIF_TEST_JACOBIANS(R7f);
MANIF_TEST_JACOBIANS(R8f);
MANIF_TEST_JACOBIANS(R9f);

MANIF_TEST(R1d);
MANIF_TEST(R2d);
MANIF_TEST(R3d);
MANIF_TEST(R4d);
MANIF_TEST(R5d);
MANIF_TEST(R6d);
MANIF_TEST(R7d);
MANIF_TEST(R8d);
MANIF_TEST(R9d);

MANIF_TEST_MAP(R1d);
MANIF_TEST_MAP(R2d);
MANIF_TEST_MAP(R3d);
MANIF_TEST_MAP(R4d);
MANIF_TEST_MAP(R5d);
MANIF_TEST_MAP(R6d);
MANIF_TEST_MAP(R7d);
MANIF_TEST_MAP(R8d);
MANIF_TEST_MAP(R9d);

MANIF_TEST_JACOBIANS(R1d);
MANIF_TEST_JACOBIANS(R2d);
MANIF_TEST_JACOBIANS(R3d);
MANIF_TEST_JACOBIANS(R4d);
MANIF_TEST_JACOBIANS(R5d);
MANIF_TEST_JACOBIANS(R6d);
MANIF_TEST_JACOBIANS(R7d);
MANIF_TEST_JACOBIANS(R8d);
MANIF_TEST_JACOBIANS(R9d);

#endif

MANIF_RUN_ALL_TEST;
