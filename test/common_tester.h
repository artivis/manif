#ifndef _MANIF_MANIF_TEST_COMMON_TESTER_H_
#define _MANIF_MANIF_TEST_COMMON_TESTER_H_

#include "gtest_manif_utils.h"
#include "test_func.h"
#include "manif/algorithms/interpolation.h"
#include "manif/algorithms/average.h"

#define MANIF_TEST(manifold)                                              \
  using TEST_##manifold##_TESTER = CommonTester<manifold>;                \
  TEST_F(TEST_##manifold##_TESTER, TEST_##manifold##_MISC)                \
  { evalMisc(); }                                                         \
  TEST_F(TEST_##manifold##_TESTER, TEST_##manifold##_COPY_CONSTRUCTOR)    \
  { evalCopyConstructor(); }                                              \
  TEST_F(TEST_##manifold##_TESTER, TEST_##manifold##_MOVE_CONSTRUCTOR)    \
  { evalMoveConstructor(); }                                              \
  TEST_F(TEST_##manifold##_TESTER, TEST_##manifold##_ASSIGNMENT)          \
  { evalAssignment(); }                                                   \
  TEST_F(TEST_##manifold##_TESTER, TEST_##manifold##_MOVE_ASSIGNMENT)     \
  { evalMoveAssignment(); }                                               \
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
  { evalRetrLift(); }                                                     \
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
  TEST_F(TEST_##manifold##_TESTER, TEST_##manifold##_INTERP)              \
  { evalInterp(); }                                                       \
  TEST_F(TEST_##manifold##_TESTER, TEST_##manifold##_AVG_BIINVARIANT)     \
  { evalAvgBiInvariant(); }                                               \
  TEST_F(TEST_##manifold##_TESTER, TEST_##manifold##_IS_APPROX)           \
  { evalIsApprox(); }                                                     \
  TEST_F(TEST_##manifold##_TESTER, TEST_##manifold##_UNARY_MINUS)         \
  { evalUnaryMinus(); }                                                   \
  TEST_F(TEST_##manifold##_TESTER, TEST_##manifold##_TANGENT_OPERATORS)   \
  { evalSomeTangentOperators(); }                                         \
  TEST_F(TEST_##manifold##_TESTER, TEST_##manifold##_GENERATORS_HAT)      \
  { evalGeneratorsHat(); }                                                \
  TEST_F(TEST_##manifold##_TESTER, TEST_##manifold##_STREAM_OP)           \
  { evalStreamOp(); }                                                     \
  TEST_F(TEST_##manifold##_TESTER, TEST_##manifold##_INNER)               \
  { evalInner(); }                                                        \
  TEST_F(TEST_##manifold##_TESTER, TEST_##manifold##_NUMERICAL_STABILITY) \
  { evalNumericalStability(); }                                           \
  TEST_F(TEST_##manifold##_TESTER, TEST_##manifold##_SMALL_ADJ)           \
  { evalSmallAdj(); }                                                     \
  TEST_F(TEST_##manifold##_TESTER, TEST_##manifold##_IDENTITY_ACT_POINT)  \
  { evalIdentityActPoint(); }

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
  { evalJrJrinvJlJlinv(); }                                                       \
  TEST_F(TEST_##manifold##_JACOBIANS_TESTER, TEST_##manifold##_ACT_JACOBIANS)     \
  { evalActJac(); }                                                               \
  TEST_F(TEST_##manifold##_JACOBIANS_TESTER, TEST_##manifold##_PLUS_T_JACOBIANS)  \
  { evalTanPlusTanJac(); }                                                        \
  TEST_F(TEST_##manifold##_JACOBIANS_TESTER, TEST_##manifold##_MINUS_T_JACOBIANS) \
  { evalTanMinusTanJac(); }

#define MANIF_TEST_MAP(manifold)                                          \
  using TEST_##manifold##_MAP_TESTER = CommonMapTester<manifold>;         \
  TEST_F(TEST_##manifold##_MAP_TESTER, TEST_##manifold##_DATA_PTR)        \
  { evalDataPtr(); }                                                      \
  TEST_F(TEST_##manifold##_MAP_TESTER, TEST_##manifold##_ASSIGN_OP)       \
  { evalAssignOp(); }                                                     \
  TEST_F(TEST_##manifold##_MAP_TESTER, TEST_##manifold##_MOVE_ASSIGN_OP)  \
  { evalMoveAssignOp(); }                                                 \
  TEST_F(TEST_##manifold##_MAP_TESTER, TEST_##manifold##_SET_RANDOM)      \
  { evalSetRandom(); }                                                    \
  TEST_F(TEST_##manifold##_MAP_TESTER, TEST_##manifold##_SET_IDENTITY)    \
  { evalSetIdentity(); }


namespace manif {

/**
 * @brief A helper class to test some common functionalities
 */
template <typename _LieGroup>
class CommonTester : public ::testing::Test
{
  using LieGroup = _LieGroup;
  using Scalar   = typename LieGroup::Scalar;
  using Tangent  = typename LieGroup::Tangent;

public:

  MANIF_MAKE_ALIGNED_OPERATOR_NEW_COND_TYPE(LieGroup)

  CommonTester()  = default;
  ~CommonTester() = default;

  void SetUp() override
  {
    std::srand((unsigned int) time(0));

    state       = LieGroup::Random();
    state_other = LieGroup::Random();

    delta = Tangent::Random();
  }

  void evalMisc()
  {
    if (std::is_nothrow_move_constructible<Scalar>::value)
    {
      EXPECT_TRUE(std::is_nothrow_move_constructible<LieGroup>::value)
        << "LieGroup should be nothrow move constructible";
      EXPECT_TRUE(std::is_nothrow_move_constructible<Tangent>::value)
        << "Tangent should be nothrow move constructible";
    }
  }

  void evalCopyConstructor()
  {
    LieGroup state_copy(state);
    EXPECT_MANIF_NEAR(state, state_copy, tol_);
  }

  void evalMoveConstructor()
  {
    LieGroup state_copy(state);
    LieGroup state_move(std::move(state));
    EXPECT_MANIF_NEAR(state_copy, state_move, tol_);
  }

  void evalAssignment()
  {
    LieGroup state_copy;
    state_copy = state;
    EXPECT_MANIF_NEAR(state, state_copy, tol_);

    state_copy = LieGroup::Random();
    EXPECT_MANIF_NOT_NEAR(state, state_copy, tol_);

    // copy derived to base
    copy_assign(state_copy, state);
    EXPECT_MANIF_NEAR(state, state_copy, tol_);

    state_copy = LieGroup::Random();
    EXPECT_MANIF_NOT_NEAR(state, state_copy, tol_);

    // copy base to base
    copy_assign_base(state_copy, state);
    EXPECT_MANIF_NEAR(state, state_copy, tol_);
  }

  void evalMoveAssignment()
  {
    LieGroup state_copy(state);
    LieGroup state_move = std::move(state);
    EXPECT_MANIF_NEAR(state_copy, state_move, tol_);

    state_move = LieGroup::Random();
    EXPECT_MANIF_NOT_NEAR(state_copy, state_move, tol_);
    state = state_copy;
    EXPECT_MANIF_NEAR(state_copy, state, tol_);

    // move derived to base
    move_assign(state_move, state);
    EXPECT_MANIF_NEAR(state_copy, state_move, tol_);

    state_move = LieGroup::Random();
    EXPECT_MANIF_NOT_NEAR(state, state_move, tol_);
    state = state_copy;
    EXPECT_MANIF_NEAR(state_copy, state, tol_);

    // move base to base
    move_assign_base(state_move, state);
    EXPECT_MANIF_NEAR(state_copy, state_move, tol_);
  }

  void evalDataPtrValid()
  {
    ASSERT_NE(nullptr, state.data());

    const typename LieGroup::Scalar* data = state.data();

    ASSERT_NE(nullptr, data);
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
    LieGroup ret = state;
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
    LieGroup ret = state;
    ret *= state_other;

    EXPECT_MANIF_NEAR(state * state_other, ret, tol_);
  }

  void evalLiftRetr()
  {
    EXPECT_MANIF_NEAR(state, state.log().exp(), tol_);
  }

  void evalRetrLift()
  {
    EXPECT_MANIF_NEAR(delta, delta.exp().log(), tol_);
  }

  void evalComposeWithInv()
  {
    EXPECT_MANIF_NEAR(LieGroup::Identity(),
                      state.compose(state.inverse()), tol_);
    EXPECT_MANIF_NEAR(LieGroup::Identity(),
                      state * state.inverse(), tol_);

    LieGroup tmp = state; tmp *= state.inverse();
    EXPECT_MANIF_NEAR(LieGroup::Identity(), tmp, tol_);

    EXPECT_MANIF_NEAR(state, state * LieGroup::Identity(), tol_);
    EXPECT_MANIF_NEAR(state, LieGroup::Identity() * state, tol_);
  }

  void evalBetweenSelf()
  {
    EXPECT_MANIF_NEAR(LieGroup::Identity(),
                      state.between(state), tol_);
  }

  void evalPlusZero()
  {
    EXPECT_MANIF_NEAR(LieGroup::Identity(),
                      LieGroup::Identity() + Tangent::Zero(), tol_);
    EXPECT_MANIF_NEAR(state,
                      state + Tangent::Zero(), tol_);
  }

  void evalMinusSelf()
  {
    EXPECT_MANIF_NEAR(Tangent::Zero(),
                      LieGroup::Identity() - LieGroup::Identity(), tol_);
    EXPECT_MANIF_NEAR(Tangent::Zero(),
                      state - state, tol_);
  }

  void evalRetrZero()
  {
    EXPECT_MANIF_NEAR(LieGroup::Identity(),
                      Tangent::Zero().exp(), tol_);
  }

  void evalLiftIdentity()
  {
    EXPECT_MANIF_NEAR(Tangent::Zero(),
                      LieGroup::Identity().log(), tol_);

    Tangent t; t.setZero();
    LieGroup l; l.setIdentity();
    EXPECT_MANIF_NEAR(t, l.log(), tol_);
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

  void evalInterp()
  {
    EXPECT_THROW(smoothing_phi(0, 0), std::logic_error);
    EXPECT_THROW(smoothing_phi(0, 5), std::logic_error);

    for (int i=1;i<5; ++i)
    {
      EXPECT_NEAR(0, smoothing_phi(0., i), 1e-10);
      EXPECT_NEAR(1, smoothing_phi(1., i), 1e-10);
    }

    EXPECT_THROW(interpolate(state, state_other, 0.-1e-3), std::runtime_error);
    EXPECT_THROW(interpolate(state, state_other, 1.+1e-3), std::runtime_error);

    EXPECT_THROW(interpolate_smooth(state, state_other, 1.+1e-3, 0),
                 std::runtime_error);

    for (int i=0; i<3; ++i)
    {
      /// @todo cubic is faulty, need fix
      if (i==1) continue;

      const manif::INTERP_METHOD method = static_cast<INTERP_METHOD>(i);

      LieGroup interp = interpolate(state, state_other, 0, method);

      EXPECT_MANIF_NEAR(state, interp, tol_); // @todo(artivis) Fix msvc << double(i);

      interp = interpolate(state, state_other, 1, method);

      EXPECT_MANIF_NEAR(state_other, interp, tol_); // @todo(artivis) Fix msvc << double(i);
    }
  }

  void evalAvgBiInvariant()
  {
    // Note : average works over points that are 'close'.

    EXPECT_THROW(average_biinvariant(std::vector<LieGroup>{}),
                 std::runtime_error);

    {
      const auto dummy = LieGroup::Random();
      std::vector<LieGroup> tmp;
      tmp.push_back(dummy);
      EXPECT_MANIF_NEAR(dummy, average_biinvariant(tmp), tol_);
    }

    const LieGroup centroid = LieGroup::Random();

    std::vector<LieGroup> mans;

    // Generate N points arround the centroid
    const int N = 15;
    for (int i=0; i<N; ++i)
    {
      mans.push_back(
            centroid + (Tangent::Random()*0.25)
            );
    }

    const auto avg = average_biinvariant(mans);

    // A proper mean function should always return
    // the same mean no matter the initial pivot.
    for (int i=0; i<20; ++i)
    {
      std::random_shuffle( mans.begin(), mans.end() );

      const auto avg_shu = average_biinvariant(mans);

      EXPECT_MANIF_NEAR(avg, avg_shu, (std::is_same<Scalar, float>::value)? 1e-3 : 1e-5);
    }
  }

  void evalIsApprox()
  {
    // Group

    EXPECT_TRUE(LieGroup::Identity().isApprox(LieGroup::Identity(), tol_));
    EXPECT_FALSE(state.isApprox(LieGroup::Identity(), tol_));

    EXPECT_TRUE(LieGroup::Identity() == LieGroup::Identity());
    EXPECT_FALSE(state == LieGroup::Identity());

    EXPECT_TRUE(state.isApprox(state, tol_));
    EXPECT_FALSE(state.isApprox(state_other, tol_));

    // cppcheck-suppress duplicateExpression
    EXPECT_TRUE(state == state);
    EXPECT_FALSE(state == state_other);

    // Tangent

    EXPECT_TRUE(Tangent::Zero().isApprox(Tangent::Zero(), tol_));
    EXPECT_FALSE(delta.isApprox(Tangent::Zero(), tol_));

    EXPECT_TRUE(Tangent::Zero() == Tangent::Zero());
    EXPECT_FALSE(delta == Tangent::Zero());

    EXPECT_TRUE(delta.isApprox(delta, tol_));
    EXPECT_FALSE(delta.isApprox(delta+delta, tol_));

    // cppcheck-suppress duplicateExpression
    EXPECT_TRUE(delta == delta);
    EXPECT_FALSE(delta == (delta+delta));
  }

  void evalUnaryMinus()
  {
    Tangent minus_delta = -delta;
    typename Tangent::DataType delta_data = delta.coeffs();
    typename Tangent::DataType minus_delta_data = minus_delta.coeffs();
    EXPECT_EIGEN_NEAR(-delta_data, minus_delta_data);
    EXPECT_EIGEN_NEAR(-delta.coeffs(), minus_delta.coeffs());
  }

  void evalSomeTangentOperators()
  {
    typename Tangent::DataType delta_data(delta.coeffs());

    // t+t
    EXPECT_EIGEN_NEAR(delta.plus(delta).coeffs(), delta_data+delta_data);

    // t-t
    EXPECT_EIGEN_NEAR(delta.minus(delta).coeffs(), delta_data-delta_data);

    // t+t
    EXPECT_EIGEN_NEAR((delta+delta).coeffs(), delta_data+delta_data);

    // t-t
    EXPECT_EIGEN_NEAR((delta-delta).coeffs(), delta_data-delta_data);

    // t+=t
    EXPECT_EIGEN_NEAR((delta+=delta).coeffs(), delta_data+=delta_data);

    // t-=t
    EXPECT_EIGEN_NEAR((delta-=delta).coeffs(), delta_data-=delta_data);

    // t+v
    EXPECT_EIGEN_NEAR((delta+delta_data).coeffs(), delta_data+delta_data);

    // t-v
    EXPECT_EIGEN_NEAR((delta-delta_data).coeffs(), delta_data-delta_data);

    // t+=v
    EXPECT_EIGEN_NEAR((delta+=delta_data).coeffs(), delta_data+=delta_data);

    // t-=v
    EXPECT_EIGEN_NEAR((delta-=delta_data).coeffs(), delta_data-=delta_data);

    // v+t
    EXPECT_EIGEN_NEAR(delta_data+delta, delta_data+delta_data);

    // v-t
    EXPECT_EIGEN_NEAR(delta_data-delta, delta_data-delta_data);

    // ret type is Tangent
    EXPECT_EIGEN_NEAR(
          (delta+delta+delta-delta+delta_data-delta_data+delta/2.+delta*3.).coeffs(),
          delta_data+delta_data+delta_data-delta_data+delta_data-delta_data+delta_data/2.+delta_data*3.
          );

    // ret type is Tangent::DataType
    EXPECT_EIGEN_NEAR(
          delta_data+delta+delta-delta+delta_data-delta_data+delta/2.+delta*3.,
          delta_data+delta_data+delta_data-delta_data+delta_data-delta_data+delta_data/2.+delta_data*3.
          );

    Tangent w;
    w << delta_data;

    EXPECT_EIGEN_NEAR(delta_data, w.coeffs());

    EXPECT_EIGEN_NEAR((delta*3.14).coeffs(), delta_data*3.14);
    EXPECT_EIGEN_NEAR((3.14*delta).coeffs(), 3.14*delta_data);
    EXPECT_EIGEN_NEAR((delta*=3.14).coeffs(), delta_data*=3.14);

    EXPECT_EIGEN_NEAR((delta/5.12).coeffs(), delta_data/5.12);
    EXPECT_EIGEN_NEAR((delta/=5.12).coeffs(), delta_data/=5.12);
  }

  void evalGeneratorsHat()
  {
    EXPECT_THROW(Tangent::Generator(-1), manif::invalid_argument);
    EXPECT_THROW(Tangent::Generator(42), manif::invalid_argument);

    typename Tangent::LieAlg sum_delta_hat;
    sum_delta_hat.setZero();

    for (int i=0; i<Tangent::DoF; ++i)
    {
      sum_delta_hat += delta.coeffs()(i) * Tangent::Generator(i);
    }

    EXPECT_EIGEN_NEAR(delta.hat(), sum_delta_hat);

    sum_delta_hat.setZero();
    for (int i=0; i<Tangent::DoF; ++i)
    {
      sum_delta_hat += delta.coeffs()(i) * delta.generator(i);
    }

    EXPECT_EIGEN_NEAR(delta.hat(), sum_delta_hat);
  }

  void evalStreamOp()
  {
    {
      std::stringstream ss;
      ss << state;

      EXPECT_FALSE(ss.str().empty());
    }

    {
      std::stringstream ss;
      ss << delta;

      EXPECT_FALSE(ss.str().empty());
    }
  }

  void evalInner()
  {
    EXPECT_DOUBLE_EQ(0., Tangent::Zero().weightedNorm());
    EXPECT_DOUBLE_EQ(0., Tangent::Zero().squaredWeightedNorm());
    EXPECT_DOUBLE_EQ(0., Tangent::Zero().inner(Tangent::Zero()));

    Tangent delta_other = Tangent::Random();

    EXPECT_DOUBLE_EQ(delta.squaredWeightedNorm(), delta.inner(delta));
    EXPECT_DOUBLE_EQ(delta.inner(delta_other), delta_other.inner(delta));
  }

  void evalNumericalStability()
  {
    unsigned int i = 0;
    EXPECT_NO_THROW(
      for (; i < 10000; ++i) {
        state += Tangent::Random();
      }
    ) << "+= failed at iteration " << i ;
  }

  void evalSmallAdj()
  {
    const Tangent delta_other = Tangent::Random();

    EXPECT_EIGEN_NEAR((delta.smallAdj() * delta_other).hat(),
                      delta.hat() * delta_other.hat() - delta_other.hat() * delta.hat());
  }

  void evalIdentityActPoint()
  {
    using Point = typename LieGroup::Vector;

    Point pin = Point::Random();

    Point pout = LieGroup::Identity().act(pin);

    EXPECT_EIGEN_NEAR(pin, pout);
  }

protected:

  // relax eps for float type
  Scalar tol_ = (std::is_same<Scalar, float>::value)? 1e-6 : 1e-8;

  LieGroup state;
  LieGroup state_other;
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
 * - log
 * - exp
 * - compose
 * - between
 * - rplus  / lplus  / plus
 * - rminus / lminus / minus
 *
 */
template <typename _LieGroup>
class JacobianTester : public ::testing::Test
{
  using LieGroup = _LieGroup;
  using Scalar   = typename LieGroup::Scalar;
  using Tangent  = typename LieGroup::Tangent;
  using Jacobian = typename LieGroup::Jacobian;

public:

  MANIF_MAKE_ALIGNED_OPERATOR_NEW_COND_TYPE(LieGroup)

  JacobianTester()  = default;
  ~JacobianTester() = default;

  void SetUp() override
  {
    std::srand((unsigned int) time(0));

    state       = LieGroup::Random();
    state_other = LieGroup::Random();

    delta = Tangent::Random();
    w     = Tangent(Tangent::DataType::Random()*w_order_);
  }

  void evalInverseJac()
  {
    LieGroup state_out = state.inverse(J_out_lhs);

    LieGroup state_pert = (state+w).inverse();
    LieGroup state_lin  = state_out.rplus(J_out_lhs*w);

    EXPECT_MANIF_NEAR(state_pert, state_lin, tol_);
  }

  void evalLiftJac()
  {
    Tangent state_out = state.log(J_out_lhs);

    Tangent state_pert = (state+w).log();
    Tangent state_lin  = state_out + (J_out_lhs*w);

    EXPECT_MANIF_NEAR(state_pert, state_lin, tol_);
  }

  void evalRetractJac()
  {
    LieGroup state_out = delta.exp(J_out_lhs);

    LieGroup state_pert = (delta+w).exp();
    LieGroup state_lin  = state_out + (J_out_lhs*w);

    EXPECT_MANIF_NEAR(state_pert, state_lin, tol_);

    ///////

    delta.setZero();
    state_out = delta.exp(J_out_lhs);

    state_pert = (delta+w).exp();
    state_lin  = state_out + (J_out_lhs*w);

    EXPECT_MANIF_NEAR(state_pert, state_lin, tol_);
  }

  void evalComposeJac()
  {
    LieGroup state_out = state.compose(state_other, J_out_lhs, J_out_rhs);

    // Jac wrt first element

    LieGroup state_pert = (state+w).compose(state_other);
    LieGroup state_lin  = state_out + J_out_lhs*w;

    EXPECT_MANIF_NEAR(state_pert, state_lin, tol_);

    // Jac wrt second element

    state_pert = state.compose(state_other+w);
    state_lin  = state_out + J_out_rhs*w;

    EXPECT_MANIF_NEAR(state_pert, state_lin, tol_);
  }

  void evalBetweenJac()
  {
    LieGroup state_out = state.between(state_other, J_out_lhs, J_out_rhs);

    // Jac wrt first element

    LieGroup state_pert = (state + w).between(state_other);
    LieGroup state_lin  = state_out + (J_out_lhs * w);

    EXPECT_MANIF_NEAR(state_pert, state_lin, tol_);

    // Jac wrt second element

    state_pert = state.between(state_other + w);
    state_lin  = state_out + (J_out_rhs * w);

    EXPECT_MANIF_NEAR(state_pert, state_lin, tol_);
  }

  void evalRplusJac()
  {
    LieGroup state_out = state.rplus(delta, J_out_lhs, J_out_rhs);

    // Jac wrt first element

    LieGroup state_pert = (state+w).rplus(delta);
    LieGroup state_lin  = state_out.rplus(J_out_lhs*w);

    EXPECT_MANIF_NEAR(state_pert, state_lin, tol_);

    // Jac wrt second element

    state_pert = state.rplus(delta+w);
    state_lin  = state_out.rplus(J_out_rhs*w);

    EXPECT_MANIF_NEAR(state_pert, state_lin, tol_);
  }

  void evalLplusJac()
  {
    LieGroup state_out = state.lplus(delta, J_out_lhs, J_out_rhs);

    // Jac wrt first element

    LieGroup state_pert = (state+w).lplus(delta);
    LieGroup state_lin  = state_out.rplus(J_out_lhs*w);

    EXPECT_MANIF_NEAR(state_pert, state_lin, tol_);

    // Jac wrt second element

    state_pert = state.lplus(delta+w);
    state_lin  = state_out.rplus(J_out_rhs*w);

    EXPECT_MANIF_NEAR(state_pert, state_lin, tol_);
  }

  void evalPlusJac()
  {
    LieGroup state_out = state.plus(delta, J_out_lhs, J_out_rhs);

    // Jac wrt first element

    LieGroup state_pert = (state+w).plus(delta);
    LieGroup state_lin  = state_out.rplus(J_out_lhs*w);

    EXPECT_MANIF_NEAR(state_pert, state_lin, tol_);

    // Jac wrt second element

    state_pert = state.plus(delta+w);
    state_lin  = state_out.rplus(J_out_rhs*w);

    EXPECT_MANIF_NEAR(state_pert, state_lin, tol_);
  }

  void evalRminusJac()
  {
    Tangent state_out = state.rminus(state_other, J_out_lhs, J_out_rhs);

    // Jac wrt first element

    Tangent state_pert = (state+w).rminus(state_other);
    Tangent state_lin  = state_out.plus(J_out_lhs*w);

    EXPECT_MANIF_NEAR(state_pert, state_lin, tol_);

    // Jac wrt second element

    state_pert = state.rminus(state_other+w);
    state_lin  = state_out.plus(J_out_rhs*w);

    EXPECT_MANIF_NEAR(state_pert, state_lin, tol_);
  }

  void evalLminusJac()
  {
    Tangent state_out = state.lminus(state_other, J_out_lhs, J_out_rhs);

    // Jac wrt first element

    Tangent state_pert = (state+w).lminus(state_other);
    Tangent state_lin  = state_out.plus(J_out_lhs*w);

    EXPECT_MANIF_NEAR(state_pert, state_lin, tol_);

    // Jac wrt second element

    state_pert = state.lminus(state_other+w);
    state_lin  = state_out.plus(J_out_rhs*w);

    EXPECT_MANIF_NEAR(state_pert, state_lin, tol_);
  }

  void evalMinusJac()
  {
    Tangent state_out = state.minus(state_other, J_out_lhs, J_out_rhs);

    // Jac wrt first element

    Tangent state_pert = (state+w).minus(state_other);
    Tangent state_lin  = state_out.plus(J_out_lhs*w);

    EXPECT_MANIF_NEAR(state_pert, state_lin, tol_);

    // Jac wrt second element

    state_pert = state.minus(state_other+w);
    state_lin  = state_out.plus(J_out_rhs*w);

    EXPECT_MANIF_NEAR(state_pert, state_lin, tol_);
  }

  void evalAdj()
  {
    typename LieGroup::Jacobian Adja, Adjb, Adjc;

    Adja = state.adj();
    Adjb = state_other.adj();
    Adjc = state.compose(state_other).adj();

    EXPECT_EIGEN_NEAR(Adja*Adjb, Adjc);

    //EXPECT_MANIF_NEAR(state.adj()*delta, (state*delta*state.inverse()).vee());
    EXPECT_MANIF_NEAR(state+delta, state.adj()*delta+state);
    EXPECT_EIGEN_NEAR(state.adj().inverse(), state.inverse().adj());
  }

  void evalAdjJlJr()
  {
    typename LieGroup::Jacobian Adj, Jr, Jl;

    Adj = state.adj();

    Tangent tan = state.log();

    Jr = tan.rjac();
    Jl = tan.ljac();

    EXPECT_EIGEN_NEAR(Jl, Adj*Jr);
    EXPECT_EIGEN_NEAR(Adj, Jl*Jr.inverse());

    // Jr(-tau) = Jl(tau)

    EXPECT_EIGEN_NEAR(Jl, (-tan).rjac());

    /////

    state.setIdentity();

    Adj = state.adj();
    tan = state.log();

    Jr = tan.rjac();
    Jl = tan.ljac();

    EXPECT_EIGEN_NEAR(Jl, Adj*Jr);
    EXPECT_EIGEN_NEAR(Adj, Jl*Jr.inverse());

    // Jr(-tau) = Jl(tau)

    EXPECT_EIGEN_NEAR(Jl, (-tan).rjac());
  }

  void evalJrJrinvJlJlinv()
  {
    using Jac = typename LieGroup::Jacobian;
    Jac Jr, Jrinv, Jl, Jlinv;

    const Tangent tan = state.log();

    Jr = tan.rjac();
    Jl = tan.ljac();

    Jrinv = tan.rjacinv();
    Jlinv = tan.ljacinv();

    EXPECT_EIGEN_NEAR(Jac::Identity(), Jr*Jrinv);
    EXPECT_EIGEN_NEAR(Jac::Identity(), Jl*Jlinv);
  }

  void evalActJac()
  {
    using Point = Eigen::Matrix<typename LieGroup::Scalar, LieGroup::Dim, 1>;
    Point point = Point::Random();

    Eigen::Matrix<typename LieGroup::Scalar, LieGroup::Dim, LieGroup::DoF> J_pout_s;
    Eigen::Matrix<typename LieGroup::Scalar, LieGroup::Dim, LieGroup::Dim> J_pout_p;

    const Point pointout = state.act(point, J_pout_s, J_pout_p);

    const Point w_point = Point::Random()*w_order_;

    // Jac wrt first element

    Point point_pert = (state+w).act(point);
    Point point_lin  = pointout + (J_pout_s*w.coeffs());

    EXPECT_EIGEN_NEAR(point_pert, point_lin, tol_);

    // Jac wrt second element

    point_pert = state.act(point+w_point);
    point_lin  = pointout + J_pout_p*w_point;

    EXPECT_EIGEN_NEAR(point_pert, point_lin, tol_);
  }

  void evalTanPlusTanJac()
  {
    const Tangent delta_other = Tangent::Random();

    const Tangent delta_out = delta.plus(delta_other, J_out_lhs, J_out_rhs);

    // Jac wrt first element

    Tangent delta_pert = (delta+w).plus(delta_other);
    Tangent delta_lin  = delta_out.plus(J_out_lhs*w);

    EXPECT_MANIF_NEAR(delta_pert, delta_lin, tol_);

    // Jac wrt second element

    delta_pert = delta.plus(delta_other+w);
    delta_lin  = delta_out.plus(J_out_rhs*w);

    EXPECT_MANIF_NEAR(delta_pert, delta_lin, tol_);
  }

  void evalTanMinusTanJac()
  {
    const Tangent delta_other = Tangent::Random();

    const Tangent delta_out = delta.minus(delta_other, J_out_lhs, J_out_rhs);

    // Jac wrt first element

    Tangent delta_pert = (delta+w).minus(delta_other);
    Tangent delta_lin  = delta_out.plus(J_out_lhs*w);

    EXPECT_MANIF_NEAR(delta_pert, delta_lin, tol_);

    // Jac wrt second element

    delta_pert = delta.minus(delta_other+w);
    delta_lin  = delta_out.plus(J_out_rhs*w);

    EXPECT_MANIF_NEAR(delta_pert, delta_lin, tol_);
  }

  void setOmegaOrder(const double w_order) { w_order_ = w_order; }
  void setTolerance(const double tol) { tol_ = tol; }

  double getOmegaOrder() const noexcept { return w_order_; }
  double getTolerance() const noexcept { return tol_; }

protected:

  double w_order_ = 1e-4;

  // relax tolerance for float type
  Scalar tol_ = (std::is_same<Scalar, float>::value)? 1e-4 : 1e-7;

  LieGroup state;
  LieGroup state_other;
  Tangent  delta;
  Tangent  w;

  Jacobian J_out_lhs, J_out_rhs;
};

template <typename _LieGroup>
class CommonMapTester : public ::testing::Test
{
  using LieGroup = _LieGroup;
  using Tangent  = typename LieGroup::Tangent;
  using Scalar   = typename LieGroup::Scalar;

  using LieGroupDataType = typename LieGroup::DataType;
  using TangentDataType  = typename Tangent::DataType;

  using MapLieGroup = Eigen::Map<LieGroup>;
  using MapTangent  = Eigen::Map<Tangent>;

public:

  MANIF_MAKE_ALIGNED_OPERATOR_NEW_COND_TYPE(LieGroup)

  CommonMapTester()
    : state_map(nullptr), state_other_map(nullptr), delta_map(nullptr) {}

  ~CommonMapTester() = default;

  void SetUp() override
  {
    std::srand((unsigned int) time(0));

    state_data.setRandom();
    state_other_data.setRandom();
    delta_data.setRandom();

    new (&state_map) Eigen::Map<LieGroup>(state_data.data());
    new (&state_other_map) Eigen::Map<LieGroup>(state_other_data.data());
    new (&delta_map) Eigen::Map<Tangent>(delta_data.data());

    ASSERT_EIGEN_NEAR(state_data, state_map.coeffs());
    ASSERT_EIGEN_NEAR(state_other_data, state_other_map.coeffs());
    ASSERT_EIGEN_NEAR(delta_data, delta_map.coeffs());

    ASSERT_EIGEN_NOT_NEAR(state_data, state_other_data);
  }

  void evalDataPtr()
  {
    Scalar* data_ptr = state_map.data();

    ASSERT_NE(nullptr, data_ptr);
    EXPECT_EQ(state_data.data(), data_ptr);

    data_ptr = delta_map.data();

    ASSERT_NE(nullptr, data_ptr);
    EXPECT_EQ(delta_data.data(), data_ptr);
  }

  void evalAssignOp()
  {
    const LieGroupDataType state_data_init = state_data;

    EXPECT_EIGEN_NEAR(state_data, state_data_init);

    state_map = state_other_map;

    EXPECT_EIGEN_NEAR(state_other_data, state_data);
  }

  void evalMoveAssignOp()
  {
    const Scalar* data_ptr = state_map.data();
    const Scalar* other_data_ptr = state_other_map.data();

    ASSERT_NE(data_ptr, other_data_ptr);

    state_map = std::move(state_other_map);

    EXPECT_EIGEN_NEAR(state_other_data, state_map.coeffs());

    EXPECT_EQ(data_ptr, state_data.data());
    EXPECT_NE(other_data_ptr, state_data.data());
  }

  void evalSetRandom()
  {
    const LieGroupDataType state_data_init = state_data;
    const TangentDataType delta_data_init = delta_data;

    EXPECT_EIGEN_NEAR(state_data, state_data_init);
    EXPECT_EIGEN_NEAR(delta_data, delta_data_init);

    state_map.setRandom();
    delta_map.setRandom();

    EXPECT_EIGEN_NOT_NEAR(state_data_init, state_data);
    EXPECT_EIGEN_NOT_NEAR(delta_data_init, delta_data);
  }

  void evalSetIdentity()
  {
    const LieGroupDataType state_data_init = state_data;
    const TangentDataType delta_data_init = delta_data;

    EXPECT_EIGEN_NEAR(state_data, state_data_init);
    EXPECT_EIGEN_NEAR(delta_data, delta_data_init);

    state_map.setIdentity();
    delta_map.setZero();

    EXPECT_EIGEN_NOT_NEAR(state_data_init, state_data);
    EXPECT_EIGEN_NOT_NEAR(delta_data_init, delta_data);

    EXPECT_EIGEN_NEAR(LieGroup::Identity().coeffs(), state_data);
    EXPECT_EIGEN_NEAR(Tangent::Zero().coeffs(), delta_data);
  }

protected:

  LieGroupDataType state_data;
  LieGroupDataType state_other_data;
  TangentDataType delta_data;

  MapLieGroup state_map;
  MapLieGroup state_other_map;

  MapTangent delta_map;
};

} /* namespace manif */

#endif /* _MANIF_MANIF_TEST_COMMON_TESTER_H_ */
