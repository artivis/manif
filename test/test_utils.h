#ifndef _MANIF_MANIF_TEST_UTILS_H_
#define _MANIF_MANIF_TEST_UTILS_H_

#include "manif/impl/manifold_base.h"
#include "manif/algorithms/interpolation.h"
#include "manif/algorithms/average.h"
#include "manif/impl/utils.h"

#include "eigen_gtest.h"

#include <random>
#include <chrono>

#define EXPECT_ANGLE_NEAR(e, a, eps) \
  EXPECT_LT(pi2pi(e-a), eps)

// https://stackoverflow.com/questions/3046889/optional-parameters-with-c-macros

#define __GET_4TH_ARG(arg1,arg2,arg3,arg4, ...) arg4

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

#define MANIF_TEST(manifold)                                              \
  using TEST_##manifold##_TESTER = CommonTester<manifold>;                \
  TEST_F(TEST_##manifold##_TESTER, TEST_##manifold##_COPY_CONSTRUCTOR)    \
  { evalCopyConstructor(); }                                              \
  TEST_F(TEST_##manifold##_TESTER, TEST_##manifold##_ASSIGNMENT)          \
  { evalAssignment(); }                                                   \
  TEST_F(TEST_##manifold##_TESTER, TEST_##manifold##_DATA_PTR_VALID)      \
  { evalDataPtrValid(); }                                                 \
  TEST_F(TEST_##manifold##_TESTER, TEST_##manifold##_PLUS_IS_RPLUS)       \
  { evalPlusIsRplus(); }                                                  \
  TEST_F(TEST_##manifold##_TESTER, TEST_##manifold##_PLUS_OP_IS_RPLUS)    \
  { evalPlusOpIsRplus(); }                                                \
  TEST_F(TEST_##manifold##_TESTER, TEST_##manifold##_MINUS_IS_RMINUS)     \
  { evalMinusIsRminus(); }                                                \
  TEST_F(TEST_##manifold##_TESTER, TEST_##manifold##_MINUS_OP_IS_RMINUS)  \
  { evalMinusOpIsRminus(); }                                              \
  TEST_F(TEST_##manifold##_TESTER, TEST_##manifold##_PLUS_EQ)             \
  { evalPlusEq(); }                                                       \
  TEST_F(TEST_##manifold##_TESTER, TEST_##manifold##_COMPOSE_OP)          \
  { evalCompOp(); }                                                       \
  TEST_F(TEST_##manifold##_TESTER, TEST_##manifold##_COMPOSE_EQ_OP)       \
  { evalCompEq(); }                                                       \
  TEST_F(TEST_##manifold##_TESTER, TEST_##manifold##_LIFT_RETRACT)        \
  { evalLiftRetr(); }                                                     \
  TEST_F(TEST_##manifold##_TESTER, TEST_##manifold##_RETRACT_LIFT)        \
  { evalLiftRetr(); }                                                     \
  TEST_F(TEST_##manifold##_TESTER, TEST_##manifold##_COMPOSE_WITH_INV)    \
  { evalComposeWithInv(); }                                               \
  TEST_F(TEST_##manifold##_TESTER, TEST_##manifold##_BETWEEN_SELF)        \
  { evalBetweenSelf(); }                                                  \
  TEST_F(TEST_##manifold##_TESTER, TEST_##manifold##_PLUS_ZERO)           \
  { evalPlusZero(); }                                                     \
  TEST_F(TEST_##manifold##_TESTER, TEST_##manifold##_MINUS_SELF)          \
  { evalMinusSelf(); }                                                    \
  TEST_F(TEST_##manifold##_TESTER, TEST_##manifold##_RETRACT_ZERO)        \
  { evalRetrZero(); }                                                     \
  TEST_F(TEST_##manifold##_TESTER, TEST_##manifold##_LIFT_IDENTITY)       \
  { evalLiftIdentity(); }                                                 \
  TEST_F(TEST_##manifold##_TESTER, TEST_##manifold##_RANDOM)              \
  { evalRandom(); }                                                       \
  TEST_F(TEST_##manifold##_TESTER, TEST_##manifold##_ZERO)                \
  { evalZero(); }                                                         \
  TEST_F(TEST_##manifold##_TESTER, TEST_##manifold##_SLERP10)             \
  { evalSlerp01(); }                                                      \
  TEST_F(TEST_##manifold##_TESTER, TEST_##manifold##_AVG_BIINVARIANT)     \
  { evalAvgBiInvariant(); }                                               \
  TEST_F(TEST_##manifold##_TESTER, TEST_##manifold##_IS_APPROX)           \
  { evalIsApprox(); }                                                     \
  TEST_F(TEST_##manifold##_TESTER, TEST_##manifold##_UNARY_MINUS)         \
  { evalUnaryMinus(); }

/*
  TEST_F(TEST_##manifold##_TESTER, TEST_##manifold##_CUBIC10)             \
  { evalCubic01(); }                                                      \
*/

#define MANIF_TEST_JACOBIANS(manifold)                                            \
  using TEST_##manifold##_JACOBIANS_TESTER = JacobianTester<manifold>;            \
  TEST_F(TEST_##manifold##_JACOBIANS_TESTER, TEST_##manifold##_INVERSE_JACOBIANS) \
  { evalInverseJac(); }                                                           \
  TEST_F(TEST_##manifold##_JACOBIANS_TESTER, TEST_##manifold##_LIFT_JACOBIANS)    \
  { evalLiftJac(); }                                                              \
  TEST_F(TEST_##manifold##_JACOBIANS_TESTER, TEST_##manifold##_RETRACT_JACOBIANS) \
  { evalRetractJac(); }                                                           \
  TEST_F(TEST_##manifold##_JACOBIANS_TESTER, TEST_##manifold##_RPLUS_JACOBIANS)   \
  { evalRplusJac(); }                                                             \
  TEST_F(TEST_##manifold##_JACOBIANS_TESTER, TEST_##manifold##_LPLUS_JACOBIANS)   \
  { evalLplusJac(); }                                                             \
  TEST_F(TEST_##manifold##_JACOBIANS_TESTER, TEST_##manifold##_PLUS_JACOBIANS)    \
  { evalPlusJac(); }                                                              \
  TEST_F(TEST_##manifold##_JACOBIANS_TESTER, TEST_##manifold##_RMINUS_JACOBIANS)  \
  { evalRminusJac(); }                                                            \
  TEST_F(TEST_##manifold##_JACOBIANS_TESTER, TEST_##manifold##_LMINUS_JACOBIANS)  \
  { evalLminusJac(); }                                                            \
  TEST_F(TEST_##manifold##_JACOBIANS_TESTER, TEST_##manifold##_MINUS_JACOBIANS)   \
  { evalMinusJac(); }                                                             \
  TEST_F(TEST_##manifold##_JACOBIANS_TESTER, TEST_##manifold##_COMPOSE_JACOBIANS) \
  { evalComposeJac(); }                                                           \
  TEST_F(TEST_##manifold##_JACOBIANS_TESTER, TEST_##manifold##_BETWEEN_JACOBIANS) \
  { evalBetweenJac(); }                                                           \
  TEST_F(TEST_##manifold##_JACOBIANS_TESTER, TEST_##manifold##_ADJ)               \
  { evalAdj(); }                                                                  \
  TEST_F(TEST_##manifold##_JACOBIANS_TESTER, TEST_##manifold##_ADJ_JL_JR)         \
  { evalAdjJlJr(); }                                                              \
  TEST_F(TEST_##manifold##_JACOBIANS_TESTER, TEST_##manifold##_JLJLinv_JRJRinv)   \
  { evalJrJrinvJlJlinv(); }



namespace manif {

template <class _DerivedA, class _DerivedB>
inline ::testing::AssertionResult
isManifNear(const ManifoldBase<_DerivedA>& manifold_a,
            const ManifoldBase<_DerivedB>& manifold_b,
            const std::string manifold_a_name = "manifold_a",
            const std::string manifold_b_name = "manifold_b",
            double tolerance = 1e-5)
{
  auto result =
      isEigenMatrixNear(ManifoldBase<_DerivedA>::Tangent::DataType::Zero(),
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
            const std::string tangent_a_name = "tangent_a",
            const std::string tangent_b_name = "tangent_b",
            double tolerance = 1e-5)
{
  return isEigenMatrixNear(tangent_a.coeffs(), tangent_b.coeffs(),
                           tangent_a_name, tangent_b_name,
                           tolerance);
}

/**
 * @brief A helper class to test some common functionalities
 */
template <typename _Manifold>
class CommonTester : public ::testing::Test
{
  using Manifold = _Manifold;
  using Scalar   = typename Manifold::Scalar;
  using Tangent  = typename Manifold::Tangent;

public:

  CommonTester()  = default;
  ~CommonTester() = default;

  void SetUp() override
  {
    std::srand((unsigned int) time(0));

    state       = Manifold::Random();
    state_other = Manifold::Random();

    delta = Tangent::Random();
  }

  void evalCopyConstructor()
  {
    Manifold state_copy(state);
    EXPECT_MANIF_NEAR(state, state_copy, tol_);
  }

  void evalAssignment()
  {
    Manifold state_copy;
    state_copy = state;
    EXPECT_MANIF_NEAR(state, state_copy, tol_);
  }

  void evalDataPtrValid()
  {
    ASSERT_NE(nullptr, state.data());
  }

  void evalPlusIsRplus()
  {
    EXPECT_MANIF_NEAR(state.rplus(delta),
                      state.plus(delta), tol_);
  }

  void evalPlusOpIsRplus()
  {
    EXPECT_MANIF_NEAR(state.rplus(delta),
                      state + delta, tol_);
  }

  void evalMinusIsRminus()
  {
    EXPECT_MANIF_NEAR(state.rminus(state_other),
                      state.minus(state_other), tol_);
  }

  void evalMinusOpIsRminus()
  {
    EXPECT_MANIF_NEAR(state.rminus(state_other),
                      state - state_other, tol_);
  }

  void evalPlusEq()
  {
    Manifold ret = state;
    ret += delta;

    EXPECT_MANIF_NEAR(state + delta, ret, tol_);
  }

  void evalCompOp()
  {
    EXPECT_MANIF_NEAR(state.compose(state_other),
                      state * state_other, tol_);
  }

  void evalCompEq()
  {
    Manifold ret = state;
    ret *= state_other;

    EXPECT_MANIF_NEAR(state * state_other, ret, tol_);
  }

  void evalLiftRetr()
  {
    EXPECT_MANIF_NEAR(state, state.lift().retract(), tol_);
  }

  void evalRetrLift()
  {
    EXPECT_MANIF_NEAR(delta, delta.retract().lift(), tol_);
  }

  void evalComposeWithInv()
  {
    EXPECT_MANIF_NEAR(Manifold::Identity(),
                      state.compose(state.inverse()), tol_);
    EXPECT_MANIF_NEAR(Manifold::Identity(),
                      state * state.inverse(), tol_);

    Manifold tmp = state; tmp *= state.inverse();
    EXPECT_MANIF_NEAR(Manifold::Identity(), tmp, tol_);
  }

  void evalBetweenSelf()
  {
    EXPECT_MANIF_NEAR(Manifold::Identity(),
                      state.between(state), tol_);
  }

  void evalPlusZero()
  {
    EXPECT_MANIF_NEAR(Manifold::Identity(),
                      Manifold::Identity() + Tangent::Zero(), tol_);
    EXPECT_MANIF_NEAR(state,
                      state + Tangent::Zero(), tol_);
  }

  void evalMinusSelf()
  {
    EXPECT_MANIF_NEAR(Tangent::Zero(),
                      Manifold::Identity() - Manifold::Identity(), tol_);
    EXPECT_MANIF_NEAR(Tangent::Zero(),
                      state - state, tol_);
  }

  void evalRetrZero()
  {
    EXPECT_MANIF_NEAR(Manifold::Identity(),
                      Tangent::Zero().retract(), tol_);
  }

  void evalLiftIdentity()
  {
    EXPECT_MANIF_NEAR(Tangent::Zero(),
                      Manifold::Identity().lift(), tol_);
  }

  void evalRandom()
  {
    // seed is init thus random should be different
    EXPECT_MANIF_NOT_NEAR(Tangent::Random(),
                          Tangent::Random(), tol_);

    EXPECT_MANIF_NOT_NEAR(Tangent().setRandom(),
                          Tangent().setRandom(), tol_);
  }

  void evalZero()
  {
    EXPECT_EIGEN_NEAR(Tangent::Zero().coeffs(),
                      Tangent::DataType::Zero());
  }

  void evalSlerp01()
  {
    Manifold interp = interpolate(state, state_other, 0);

    EXPECT_MANIF_NEAR(state, interp, tol_);

    interp = interpolate(state, state_other, 1);

    EXPECT_MANIF_NEAR(state_other, interp, tol_);
  }

  void evalCubic01()
  {
    Manifold interp = interpolate(state, state_other, 0, INTERP_METHOD::CUBIC);

    EXPECT_MANIF_NEAR(state, interp, tol_);

    interp = interpolate(state, state_other, 1, INTERP_METHOD::CUBIC);

    EXPECT_MANIF_NEAR(state_other, interp, tol_);
  }

  void evalAvgBiInvariant()
  {
    const auto dummy = Manifold::Random();
    EXPECT_MANIF_NEAR(dummy,
     average_biinvariant(std::vector<Manifold>{dummy}), tol_);

    std::vector<Manifold> mans;

    const int N = 15;
    for (int i=0; i<N; ++i)
      mans.emplace_back(Manifold::Random());

    const auto avg = average_biinvariant(mans);

    // A proper mean function should always return
    // the same mean no matter the initial pivot.
    for (int i=0; i<20; ++i)
    {
      std::random_shuffle( mans.begin(), mans.end() );

      const auto avg_shu = average_biinvariant(mans);

      EXPECT_MANIF_NEAR(avg, avg_shu, 1e-8);
    }
  }

  void evalIsApprox()
  {
    EXPECT_TRUE(state.isApprox(state, tol_));
    EXPECT_FALSE(state.isApprox(state_other, tol_));

    EXPECT_TRUE(state == state);
    EXPECT_FALSE(state == state_other);
  }

  void evalUnaryMinus()
  {
    Tangent minus_delta = -delta;
    typename Tangent::DataType delta_data = delta.coeffs();
    typename Tangent::DataType minus_delta_data = minus_delta.coeffs();
    EXPECT_EIGEN_NEAR(-delta_data, minus_delta_data);
    EXPECT_EIGEN_NEAR(-delta.coeffs(), minus_delta.coeffs());
  }

protected:

  Scalar tol_ = Constants<Scalar>::eps;

  Manifold state;
  Manifold state_other;
  Tangent  delta;
};

/**
 * @brief A helper class to test general Jacobians
 *
 * assert_near(f(R.plus(w)) , f(R).plus(J_f_R * w)
 * assert_near(f(R.plus(w)) , f(R) + (J_f_R * w)
 *
 * GENERAL JACOBIANS
 *
 * Assert is through linearized equation
 * J = df/dR ==> f( R(+)w ) ~= f(R) (+) J*w, where (+) is right-plus
 *
 * The Jacobians of the following operations are currently tested :
 *
 * - inverse
 * - lift
 * - retract
 * - compose
 * - between
 * - rplus  / lplus  / plus
 * - rminus / lminus / minus
 *
 */
template <typename _Manifold>
class JacobianTester : public ::testing::Test
{
  using Manifold = _Manifold;
  using Tangent  = typename _Manifold::Tangent;

public:

  JacobianTester()  = default;
  ~JacobianTester() = default;

  void SetUp() override
  {
    std::srand((unsigned int) time(0));

    state       = Manifold::Random();
    state_other = Manifold::Random();

    delta = Tangent::Random();
    w     = Tangent(Tangent::DataType::Random()*w_order_);
  }

  void evalInverseJac()
  {
    typename Manifold::Jacobian J_sout_s;
    Manifold state_out = state.inverse(J_sout_s);

    Manifold state_pert = (state+w).inverse();
    Manifold state_lin  = state_out.rplus(J_sout_s*w);

    EXPECT_MANIF_NEAR(state_pert, state_lin, tol_);
  }

  void evalLiftJac()
  {
    typename Manifold::Jacobian J_sout_s;
    Tangent state_out = state.lift(J_sout_s);

    Tangent state_pert = (state+w).lift();
    Tangent state_lin  = state_out + (J_sout_s*w);

    EXPECT_MANIF_NEAR(state_pert, state_lin, tol_);
  }

  void evalRetractJac()
  {
    typename Manifold::Jacobian J_sout_s;
    Manifold state_out = delta.retract(J_sout_s);

    Manifold state_pert = (delta+w).retract();
    Manifold state_lin  = state_out + (J_sout_s*w);

    EXPECT_MANIF_NEAR(state_pert, state_lin, tol_);
  }

  void evalComposeJac()
  {
    typename Manifold::Jacobian J_sout_s, J_sout_so;
    Manifold state_out = state.compose(state_other, J_sout_s, J_sout_so);

    // Jac wrt first element

    Manifold state_pert = (state+w).compose(state_other);
    Manifold state_lin  = state_out + J_sout_s*w;

    EXPECT_MANIF_NEAR(state_pert, state_lin, tol_);

    // Jac wrt second element

    state_pert = state.compose(state_other+w);
    state_lin  = state_out + J_sout_so*w;

    EXPECT_MANIF_NEAR(state_pert, state_lin, tol_);
  }

  void evalBetweenJac()
  {
    typename Manifold::Jacobian J_sout_s, J_sout_so;
    Manifold state_out = state.between(state_other, J_sout_s, J_sout_so);

    // Jac wrt first element

    Manifold state_pert = (state + w).between(state_other);
    Manifold state_lin  = state_out + (J_sout_s * w);

    EXPECT_MANIF_NEAR(state_pert, state_lin, tol_);

    // Jac wrt second element

    state_pert = state.between(state_other + w);
    state_lin  = state_out + (J_sout_so * w);

    EXPECT_MANIF_NEAR(state_pert, state_lin, tol_);
  }

  void evalRplusJac()
  {
    typename Manifold::Jacobian J_sout_s, J_sout_t;
    Manifold state_out = state.rplus(delta, J_sout_s, J_sout_t);

    // Jac wrt first element

    Manifold state_pert = (state+w).rplus(delta);
    Manifold state_lin  = state_out.rplus(J_sout_s*w);

    EXPECT_MANIF_NEAR(state_pert, state_lin, tol_);

    // Jac wrt second element

    state_pert = state.rplus(delta+w);
    state_lin  = state_out.rplus(J_sout_t*w);

    EXPECT_MANIF_NEAR(state_pert, state_lin, tol_);
  }

  void evalLplusJac()
  {
    typename Manifold::Jacobian J_sout_s, J_sout_t;
    Manifold state_out = state.lplus(delta, J_sout_s, J_sout_t);

    // Jac wrt first element

    Manifold state_pert = (state+w).lplus(delta);
    Manifold state_lin  = state_out.rplus(J_sout_s*w);

    EXPECT_MANIF_NEAR(state_pert, state_lin, tol_);

    // Jac wrt second element

    state_pert = state.lplus(delta+w);
    state_lin  = state_out.rplus(J_sout_t*w);

    EXPECT_MANIF_NEAR(state_pert, state_lin, tol_);
  }

  void evalPlusJac()
  {
    typename Manifold::Jacobian J_sout_s, J_mout_t;
    Manifold state_out = state.plus(delta, J_sout_s, J_mout_t);

    // Jac wrt first element

    Manifold state_pert = (state+w).plus(delta);
    Manifold state_lin  = state_out.rplus(J_sout_s*w);

    EXPECT_MANIF_NEAR(state_pert, state_lin, tol_);

    // Jac wrt second element

    state_pert = state.plus(delta+w);
    state_lin  = state_out.rplus(J_mout_t*w);

    EXPECT_MANIF_NEAR(state_pert, state_lin, tol_);
  }

  void evalRminusJac()
  {
    typename Manifold::Jacobian J_sout_s, J_mout_so;
    Tangent state_out = state.rminus(state_other, J_sout_s, J_mout_so);

    // Jac wrt first element

    Tangent state_pert = (state+w).rminus(state_other);
    Tangent state_lin  = state_out.plus(J_sout_s*w);

    EXPECT_MANIF_NEAR(state_pert, state_lin, tol_);

    // Jac wrt second element

    state_pert = state.rminus(state_other+w);
    state_lin  = state_out.plus(J_mout_so*w);

    EXPECT_MANIF_NEAR(state_pert, state_lin, tol_);
  }

  void evalLminusJac()
  {
    typename Manifold::Jacobian J_sout_s, J_mout_so;
    Tangent state_out = state.lminus(state_other, J_sout_s, J_mout_so);

    // Jac wrt first element

    Tangent state_pert = (state+w).lminus(state_other);
    Tangent state_lin  = state_out.plus(J_sout_s*w);

    EXPECT_MANIF_NEAR(state_pert, state_lin, tol_);

    // Jac wrt second element

    state_pert = state.lminus(state_other+w);
    state_lin  = state_out.plus(J_mout_so*w);

    EXPECT_MANIF_NEAR(state_pert, state_lin, tol_);
  }

  void evalMinusJac()
  {
    typename Manifold::Jacobian J_sout_s, J_mout_so;
    Tangent state_out = state.minus(state_other, J_sout_s, J_mout_so);

    // Jac wrt first element

    Tangent state_pert = (state+w).minus(state_other);
    Tangent state_lin  = state_out.plus(J_sout_s*w);

    EXPECT_MANIF_NEAR(state_pert, state_lin, tol_);

    // Jac wrt second element

    state_pert = state.minus(state_other+w);
    state_lin  = state_out.plus(J_mout_so*w);

    EXPECT_MANIF_NEAR(state_pert, state_lin, tol_);
  }

  void evalAdj()
  {
    typename Manifold::Jacobian Adja, Adjb, Adjc;

    Adja = state.adj();
    Adjb = state_other.adj();
    Adjc = state.compose(state_other).adj();

    EXPECT_EIGEN_NEAR(Adja*Adjb, Adjc);
  }

  void evalAdjJlJr()
  {
    typename Manifold::Jacobian Adj, Jr, Jl;

    Adj = state.adj();

    const Tangent tan = state.lift();

    Jr = tan.rjac();
    Jl = tan.ljac();

    EXPECT_EIGEN_NEAR(Jl, Adj*Jr);
    EXPECT_EIGEN_NEAR(Adj, Jl*Jr.inverse());
  }

  void evalJrJrinvJlJlinv()
  {
    using Jac = typename Manifold::Jacobian;
    Jac Jr, Jrinv, Jl, Jlinv;

    const Tangent tan = state.lift();

    Jr = tan.rjac();
    Jl = tan.ljac();

    Jrinv = tan.rjacinv();
    Jlinv = tan.ljacinv();

    EXPECT_EIGEN_NEAR(Jac::Identity(), Jr*Jrinv);
    EXPECT_EIGEN_NEAR(Jac::Identity(), Jl*Jlinv);
  }

  void setOmegaOrder(const double w_order) { w_order_ = w_order; }
  void setTolerance(const double tol) { tol_ = tol; }

  double getOmegaOrder() const noexcept { return w_order_; }
  double getTolerance() const noexcept { return tol_; }

protected:

  double w_order_ = 1e-4;
  double tol_ = 1e-8;

  Manifold state;
  Manifold state_other;
  Tangent  delta;
  Tangent  w; //
};

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

template <typename _Derived>
const typename _Derived::DataType&
callCoeffs(const ManifoldBase<_Derived>& manifold)
{
  return manifold.coeffs();
}

template <typename _Derived>
const typename _Derived::DataType&
callCoeffs(const TangentBase<_Derived>& tangent)
{
  return tangent.coeffs();
}

template <typename _Derived>
typename _Derived::Transformation
callTransform(const ManifoldBase<_Derived>& manifold)
{
  return manifold.transform();
}

template <typename _Derived>
typename _Derived::Rotation
callRotation(const ManifoldBase<_Derived>& manifold)
{
  return manifold.rotation();
}

template <typename _Derived>
void
callIdentity(ManifoldBase<_Derived>& manifold)
{
  manifold.setIdentity();
}

template <typename _Derived>
void
callZero(TangentBase<_Derived>& tangent)
{
  tangent.setZero();
}

template <typename _Derived>
void
callRandom(ManifoldBase<_Derived>& manifold)
{
  manifold.setRandom();
}

template <typename _Derived>
void
callRandom(TangentBase<_Derived>& tangent)
{
  tangent.setRandom();
}

template <typename _Derived>
typename _Derived::Manifold
callInverse(const ManifoldBase<_Derived>& manifold)
{
  return manifold.inverse();
}

template <typename _DerivedMan, typename _DerivedTan>
typename _DerivedMan::Manifold
callRplus(const ManifoldBase<_DerivedMan>& manifold,
          const TangentBase<_DerivedTan>& tangent)
{
  return manifold.rplus(tangent);
}

template <typename _DerivedMan, typename _DerivedTan>
typename _DerivedMan::Manifold
callLplus(const ManifoldBase<_DerivedMan>& manifold,
          const TangentBase<_DerivedTan>& tangent)
{
  return manifold.lplus(tangent);
}

template <typename _DerivedMan, typename _DerivedTan>
typename _DerivedMan::Manifold
callPlus(const ManifoldBase<_DerivedMan>& manifold,
         const TangentBase<_DerivedTan>& tangent)
{
  return manifold.plus(tangent);
}

template <typename _Derived0, typename _Derived1>
typename _Derived0::Tangent
callRminus(const ManifoldBase<_Derived0>& manifold_lhs,
           const ManifoldBase<_Derived1>& manifold_rhs)
{
  return manifold_lhs.rminus(manifold_rhs);
}

template <typename _Derived0, typename _Derived1>
typename _Derived0::Tangent
callLminus(const ManifoldBase<_Derived0>& manifold_lhs,
           const ManifoldBase<_Derived1>& manifold_rhs)
{
  return manifold_lhs.lminus(manifold_rhs);
}

template <typename _Derived0, typename _Derived1>
typename _Derived0::Tangent
callMinus(const ManifoldBase<_Derived0>& manifold_lhs,
          const ManifoldBase<_Derived1>& manifold_rhs)
{
  return manifold_lhs.minus(manifold_rhs);
}

template <typename _Derived>
typename _Derived::Tangent
callLift(const ManifoldBase<_Derived>& manifold)
{
  return manifold.lift();
}

template <typename _Derived>
typename _Derived::Manifold
callRetract(const TangentBase<_Derived>& tangent)
{
  return tangent.retract();
}

template <typename _Derived0, typename _Derived1>
typename _Derived0::Manifold
callCompose(const ManifoldBase<_Derived0>& manifold_lhs,
            const ManifoldBase<_Derived1>& manifold_rhs)
{
  return manifold_lhs.compose(manifold_rhs);
}

template <typename _Derived0, typename _Derived1>
typename _Derived0::Manifold
callBetween(const ManifoldBase<_Derived0>& manifold_lhs,
            const ManifoldBase<_Derived1>& manifold_rhs)
{
  return manifold_lhs.between(manifold_rhs);
}

template <typename _DerivedMan, typename _DerivedTan>
typename _DerivedMan::Manifold
callOpPlus(const ManifoldBase<_DerivedMan>& manifold,
           const TangentBase<_DerivedTan>& tangent)
{
  return manifold + tangent;
}

template <typename _DerivedMan, typename _DerivedTan>
void
callOpPlusEq(ManifoldBase<_DerivedMan>& manifold,
             const TangentBase<_DerivedTan>& tangent)
{
  manifold += tangent;
}

template <typename _Derived0, typename _Derived1>
typename _Derived0::Tangent
callOpMinus(const ManifoldBase<_Derived0>& manifold_lhs,
            const ManifoldBase<_Derived0>& manifold_rhs)
{
  return manifold_lhs - manifold_rhs;
}

template <typename _Derived0, typename _Derived1>
const typename _Derived0::Manifold
callOpTime(const ManifoldBase<_Derived0>& manifold_lhs,
           const ManifoldBase<_Derived1>& manifold_rhs)
{
  return manifold_lhs * manifold_rhs;
}

template <typename _Derived0, typename _Derived1>
void
callOpTimeEq(ManifoldBase<_Derived0>& manifold_lhs,
             const ManifoldBase<_Derived1>& manifold_rhs)
{
  manifold_lhs *= manifold_rhs;
}

/// with Jacs

template <typename _Derived>
typename _Derived::Manifold
callInverseWithJac(const ManifoldBase<_Derived>& manifold,
                   typename ManifoldBase<_Derived>::OptJacobianRef J_minv_m)
{
  return manifold.inverse(J_minv_m);
}

template <typename _DerivedMan, typename _DerivedTan>
const typename _DerivedMan::Manifold
callRplusWithJac(const ManifoldBase<_DerivedMan>& manifold,
                 const TangentBase<_DerivedTan>& tangent,
                 typename ManifoldBase<_DerivedMan>::OptJacobianRef J_mout_m,
                 typename TangentBase<_DerivedTan>::OptJacobianRef J_mout_t)
{
  return manifold.rplus(tangent, J_mout_m, J_mout_t);
}

template <typename _Derived>
const typename _Derived::DataType*
callLplusWithJac(const ManifoldBase<_Derived>& manifold)
{
  return manifold.data();
}

template <typename _Derived>
const typename _Derived::DataType*
callPlusWithJac(const ManifoldBase<_Derived>& manifold)
{
  return manifold.data();
}

template <typename _Derived>
const typename _Derived::DataType*
callRminusWithJac(const ManifoldBase<_Derived>& manifold)
{
  return manifold.data();
}

template <typename _Derived>
const typename _Derived::DataType*
callLminusWithJac(const ManifoldBase<_Derived>& manifold)
{
  return manifold.data();
}

template <typename _Derived>
const typename _Derived::DataType*
callMinusWithJac(const ManifoldBase<_Derived>& manifold)
{
  return manifold.data();
}

template <typename _Derived>
const typename _Derived::DataType*
callLiftWithJac(const ManifoldBase<_Derived>& manifold)
{
  return manifold.data();
}

template <typename _Derived>
typename _Derived::Manifold
callRetractWithJac(const TangentBase<_Derived>& tangent,
                   typename _Derived::Jacobian& J)
{
  return tangent.retract(J);
}

template <typename _Derived>
const typename _Derived::DataType*
callComposeWithJac(const ManifoldBase<_Derived>& manifold)
{
  return manifold.data();
}

template <typename _Derived>
const typename _Derived::DataType*
callBetweenWithJac(const ManifoldBase<_Derived>& manifold)
{
  return manifold.data();
}

} /* namespace manif */

#endif /* _MANIF_MANIF_TEST_UTILS_H_ */
