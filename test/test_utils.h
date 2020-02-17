#ifndef _MANIF_MANIF_TEST_UTILS_H_
#define _MANIF_MANIF_TEST_UTILS_H_

#include "eigen_gtest.h"

#include <random>
#include <chrono>

#define EXPECT_ANGLE_NEAR(e, a, eps) \
  EXPECT_LT(pi2pi(e-a), eps)

// https://stackoverflow.com/questions/3046889/optional-parameters-with-c-macros

#define EXPECT_MANIF_NEAR_DEFAULT_TOL(A,B) \
  EXPECT_TRUE(manif::isManifNear(A, B, #A, #B))

#define EXPECT_MANIF_NEAR_TOL(A,B,tol) \
  EXPECT_TRUE(manif::isManifNear(A, B, #A, #B, tol))

#define EXPECT_MANIF_NOT_NEAR_DEFAULT_TOL(A,B) \
  EXPECT_FALSE(manif::isManifNear(A, B, #A, #B))

#define EXPECT_MANIF_NOT_NEAR_TOL(A,B,tol) \
  EXPECT_FALSE(manif::isManifNear(A, B, #A, #B, tol))

#define __EXPECT_MANIF_NEAR_CHOOSER(...) \
  __GET_4TH_ARG(__VA_ARGS__, EXPECT_MANIF_NEAR_TOL, \
                EXPECT_MANIF_NEAR_DEFAULT_TOL, )

#define __EXPECT_MANIF_NOT_NEAR_CHOOSER(...) \
  __GET_4TH_ARG(__VA_ARGS__, EXPECT_MANIF_NOT_NEAR_TOL, \
                EXPECT_MANIF_NOT_NEAR_DEFAULT_TOL, )

#define EXPECT_MANIF_NEAR(...) \
  __EXPECT_MANIF_NEAR_CHOOSER(__VA_ARGS__)(__VA_ARGS__)

#define EXPECT_MANIF_NOT_NEAR(...) \
  __EXPECT_MANIF_NOT_NEAR_CHOOSER(__VA_ARGS__)(__VA_ARGS__)

#define ASSERT_MANIF_NEAR_DEFAULT_TOL(A,B) \
  ASSERT_TRUE(manif::isManifNear(A, B, #A, #B))

#define ASSERT_MANIF_NEAR_TOL(A,B,tol) \
  ASSERT_TRUE(manif::isManifNear(A, B, #A, #B, tol))

#define __ASSERT_MANIF_NEAR_CHOOSER(...) \
  __GET_4TH_ARG(__VA_ARGS__, ASSERT_MANIF_NEAR_TOL, \
                ASSERT_MANIF_NEAR_DEFAULT_TOL, )

#define ASSERT_MANIF_NEAR(...) \
  __ASSERT_MANIF_NEAR_CHOOSER(__VA_ARGS__)(__VA_ARGS__)

namespace manif {

template <class _DerivedA, class _DerivedB>
inline ::testing::AssertionResult
isManifNear(const LieGroupBase<_DerivedA>& manifold_a,
            const LieGroupBase<_DerivedB>& manifold_b,
            const std::string& manifold_a_name = "manifold_a",
            const std::string& manifold_b_name = "manifold_b",
            double tolerance = 1e-5)
{
  auto result =
      isEigenMatrixNear(LieGroupBase<_DerivedA>::Tangent::DataType::Zero(),
                        (manifold_a-manifold_b).coeffs(),
                        "", "", tolerance);

  return (result ? ::testing::AssertionSuccess()
                 : ::testing::AssertionFailure()
                   << manifold_a_name << " != " << manifold_b_name << "\n"
                   << manifold_a_name << ":\n" << manifold_a.coeffs().transpose() << "\n"
                   << manifold_b_name << ":\n" << manifold_b.coeffs().transpose() << "\n"
                   << "rminus:\n" << (manifold_a - manifold_b) << "\n");
}

template <class _DerivedA, class _DerivedB>
inline ::testing::AssertionResult
isManifNear(const TangentBase<_DerivedA>& tangent_a,
            const TangentBase<_DerivedB>& tangent_b,
            const std::string& tangent_a_name = "tangent_a",
            const std::string& tangent_b_name = "tangent_b",
            double tolerance = 1e-5)
{
  return isEigenMatrixNear(tangent_a.coeffs(), tangent_b.coeffs(),
                           tangent_a_name, tangent_b_name,
                           tolerance);
}

template <typename _Scalar = double>
class GaussianNoiseGenerator
{
  using Clock = std::chrono::system_clock;
  using Scalar = _Scalar;

public:

  GaussianNoiseGenerator(const Scalar mean,
                         const Scalar std)
    : re_(Clock::now().time_since_epoch().count())
    , distr_(mean, std)
  {
    //
  }

  Scalar noise()
  {
    return distr_(re_);
  }

  Scalar operator()()
  {
    return noise();
  }

protected:

  std::default_random_engine re_;
  std::normal_distribution<Scalar> distr_;
};

} /* namespace manif */

#endif /* _MANIF_MANIF_TEST_UTILS_H_ */
