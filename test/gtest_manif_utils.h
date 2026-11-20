#ifndef _MANIF_MANIF_GTEST_GTEST_MANIF_UTILS_H_
#define _MANIF_MANIF_GTEST_GTEST_MANIF_UTILS_H_

#include "manif/impl/lie_group_base.h"
#include "manif/impl/tangent_base.h"
#include "manif/impl/utils.h"

#include "gtest_eigen_utils.h"

#include <random>
#include <chrono>

#define MANIF_RUN_ALL_TEST                \
  int main(int argc, char** argv) {       \
    std::srand((unsigned int) time(0));   \
    testing::InitGoogleTest(&argc, argv); \
    return RUN_ALL_TESTS();               \
  }

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

/**
 * @brief Jacobians of a tangent evaluated from their series
 * in the small adjoint,
 *   Jr    = sum (-1)^n / (n+1)! ad^n,
 *   Jl    = sum 1 / (n+1)! ad^n,
 *   Jrinv = sum (-1)^n B_n / n! ad^n,
 *   Jlinv = sum B_n / n! ad^n,
 * where B_n are the Bernoulli numbers (B_1 = -1/2).
 * @note The series converge quickly for small angles only.
 */
template <typename _Tangent>
struct JacobiansSeries
{
  using Jacobian = typename _Tangent::Jacobian;

  explicit JacobiansSeries(const _Tangent& tangent)
    : Jr(Jacobian::Zero())
    , Jl(Jacobian::Zero())
    , Jrinv(Jacobian::Zero())
    , Jlinv(Jacobian::Zero())
  {
    const double bernoulli[] = {
      1., -1./2., 1./6., 0., -1./30., 0., 1./42., 0., -1./30., 0.,
      5./66., 0., -691./2730., 0., 7./6., 0., -3617./510.
    };

    const Jacobian ad = tangent.smallAdj();
    Jacobian ad_pow_n = Jacobian::Identity();
    double n_factorial = 1;

    for (int n = 0; n <= 16; ++n)
    {
      if (n > 0)
      {
        ad_pow_n = ad_pow_n * ad;
        n_factorial *= n;
      }

      const double sign = (n % 2 == 0) ? 1. : -1.;

      Jr    += sign / (n_factorial * (n + 1)) * ad_pow_n;
      Jl    += 1. / (n_factorial * (n + 1)) * ad_pow_n;
      Jrinv += sign * bernoulli[n] / n_factorial * ad_pow_n;
      Jlinv += bernoulli[n] / n_factorial * ad_pow_n;
    }
  }

  Jacobian Jr, Jl, Jrinv, Jlinv;
};

} /* namespace manif */

#endif /* _MANIF_MANIF_GTEST_GTEST_MANIF_UTILS_H_ */
