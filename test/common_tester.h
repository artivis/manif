#ifndef _MANIF_MANIF_TEST_COMMON_TESTER_H_
#define _MANIF_MANIF_TEST_COMMON_TESTER_H_

#include "gtest_manif_utils.h"
#include "test_func.h"
#include "manif/algorithms/interpolation.h"
#include "manif/algorithms/average.h"

#include <Eigen/StdVector>

#define MANIF_TEST(manifold)                                              \
  using TEST_##manifold##_TESTER = CommonTester<manifold>;                \
  INSTANTIATE_TEST_SUITE_P(                                               \
    TEST_##manifold##_TESTS,                                              \
    TEST_##manifold##_TESTER,                                             \
    ::testing::Values(                                                    \
      std::make_tuple(                                                    \
        manifold::Identity(),                                             \
        manifold::Identity(),                                             \
        manifold::Tangent::Zero(),                                        \
        manifold::Tangent::Zero()                                         \
      ),                                                                  \
      std::make_tuple(                                                    \
        (manifold::Tangent::Random()*1e-8).exp(),                         \
        (manifold::Tangent::Random()*1e-8).exp(),                         \
        manifold::Tangent::Random()*1e-8,                                 \
        manifold::Tangent::Random()*1e-8                                  \
      ),                                                                  \
      std::make_tuple(                                                    \
        manifold::Random(),                                               \
        manifold::Random(),                                               \
        manifold::Tangent::Random(),                                      \
        manifold::Tangent::Random()                                       \
      )                                                                   \
    ));                                                                   \
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
  TEST_P(TEST_##manifold##_TESTER, TEST_##manifold##_PLUS_IS_RPLUS)       \
  { evalPlusIsRplus(); }                                                  \
  TEST_P(TEST_##manifold##_TESTER, TEST_##manifold##_PLUS_OP_IS_RPLUS)    \
  { evalPlusOpIsRplus(); }                                                \
  TEST_P(TEST_##manifold##_TESTER, TEST_##manifold##_MINUS_IS_RMINUS)     \
  { evalMinusIsRminus(); }                                                \
  TEST_P(TEST_##manifold##_TESTER, TEST_##manifold##_MINUS_OP_IS_RMINUS)  \
  { evalMinusOpIsRminus(); }                                              \
  TEST_P(TEST_##manifold##_TESTER, TEST_##manifold##_PLUS_EQ)             \
  { evalPlusEq(); }                                                       \
  TEST_P(TEST_##manifold##_TESTER, TEST_##manifold##_COMPOSE_OP)          \
  { evalCompOp(); }                                                       \
  TEST_P(TEST_##manifold##_TESTER, TEST_##manifold##_COMPOSE_EQ_OP)       \
  { evalCompEq(); }                                                       \
  TEST_P(TEST_##manifold##_TESTER, TEST_##manifold##_LIFT_RETRACT)        \
  { evalLiftRetr(); }                                                     \
  TEST_P(TEST_##manifold##_TESTER, TEST_##manifold##_RETRACT_LIFT)        \
  { evalRetrLift(); }                                                     \
  TEST_P(TEST_##manifold##_TESTER, TEST_##manifold##_COMPOSE_WITH_INV)    \
  { evalComposeWithInv(); }                                               \
  TEST_P(TEST_##manifold##_TESTER, TEST_##manifold##_BETWEEN_SELF)        \
  { evalBetweenSelf(); }                                                  \
  TEST_P(TEST_##manifold##_TESTER, TEST_##manifold##_PLUS_ZERO)           \
  { evalPlusZero(); }                                                     \
  TEST_P(TEST_##manifold##_TESTER, TEST_##manifold##_MINUS_SELF)          \
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
  TEST_P(TEST_##manifold##_TESTER, TEST_##manifold##_IS_APPROX)           \
  { evalIsApprox(); }                                                     \
  TEST_P(TEST_##manifold##_TESTER, TEST_##manifold##_UNARY_MINUS)         \
  { evalUnaryMinus(); }                                                   \
  TEST_P(TEST_##manifold##_TESTER, TEST_##manifold##_TANGENT_OPERATORS)   \
  { evalSomeTangentOperators(); }                                         \
  TEST_P(TEST_##manifold##_TESTER, TEST_##manifold##_TANGENT_OPERATORS2)  \
  { evalSomeTangentOperators2(); }                                        \
  TEST_P(TEST_##manifold##_TESTER, TEST_##manifold##_GENERATORS_HAT)      \
  { evalGeneratorsHat(); }                                                \
  TEST_F(TEST_##manifold##_TESTER, TEST_##manifold##_STREAM_OP)           \
  { evalStreamOp(); }                                                     \
  TEST_F(TEST_##manifold##_TESTER, TEST_##manifold##_INNER_ZERO)          \
  { evalInnerZero(); }                                                    \
  TEST_P(TEST_##manifold##_TESTER, TEST_##manifold##_INNER)               \
  { evalInner(); }                                                        \
  TEST_F(TEST_##manifold##_TESTER, TEST_##manifold##_NUMERICAL_STABILITY) \
  { evalNumericalStability(); }                                           \
  TEST_P(TEST_##manifold##_TESTER, TEST_##manifold##_SMALL_ADJ)           \
  { evalSmallAdj(); }                                                     \
  TEST_F(TEST_##manifold##_TESTER, TEST_##manifold##_IDENTITY_ACT_POINT)  \
  { evalIdentityActPoint(); }                                             \
  TEST_P(TEST_##manifold##_TESTER, TEST_##manifold##_CAST)                \
  { evalCast(); }                                                         \
  TEST_P(TEST_##manifold##_TESTER, TEST_##manifold##_INVERSE)             \
  { evalInverse(); }

#define MANIF_TEST_JACOBIANS(manifold)                                                                                    \
  using manifold##JacobiansTester = JacobianTester<manifold>;                                                             \
  INSTANTIATE_TEST_SUITE_P(                                                                                               \
    manifold##JacobiansTests,                                                                                             \
    manifold##JacobiansTester,                                                                                            \
    ::testing::Values(                                                                                                    \
      std::make_tuple(                                                                                                    \
        manifold::Identity(),                                                                                             \
        manifold::Identity(),                                                                                             \
        manifold::Tangent::Zero(),                                                                                        \
        manifold::Tangent::Zero()                                                                                         \
      ),                                                                                                                  \
      std::make_tuple(                                                                                                    \
        (manifold::Tangent::Random()*1e-8).exp(),                                                                         \
        (manifold::Tangent::Random()*1e-8).exp(),                                                                         \
        manifold::Tangent::Random()*1e-8,                                                                                 \
        manifold::Tangent::Random()*1e-8                                                                                  \
      ),                                                                                                                  \
      std::make_tuple(                                                                                                    \
        manifold::Random(),                                                                                               \
        manifold::Random(),                                                                                               \
        manifold::Tangent::Random(),                                                                                      \
        manifold::Tangent::Random()                                                                                       \
      )                                                                                                                   \
    ));                                                                                                                   \
  TEST_P(manifold##JacobiansTester, TEST_##manifold##_INVERSE_JACOBIANS)                                                  \
  { evalInverseJac(); }                                                                                                   \
  TEST_P(manifold##JacobiansTester, TEST_##manifold##_LIFT_JACOBIANS)                                                     \
  { evalLiftJac(); }                                                                                                      \
  TEST_P(manifold##JacobiansTester, TEST_##manifold##_RETRACT_JACOBIANS)                                                  \
  { evalRetractJac(); }                                                                                                   \
  TEST_P(manifold##JacobiansTester, TEST_##manifold##_RPLUS_JACOBIANS)                                                    \
  { evalRplusJac(); }                                                                                                     \
  TEST_P(manifold##JacobiansTester, TEST_##manifold##_LPLUS_JACOBIANS)                                                    \
  { evalLplusJac(); }                                                                                                     \
  TEST_P(manifold##JacobiansTester, TEST_##manifold##_PLUS_JACOBIANS)                                                     \
  { evalPlusJac(); }                                                                                                      \
  TEST_P(manifold##JacobiansTester, TEST_##manifold##_RMINUS_JACOBIANS)                                                   \
  { evalRminusJac(); }                                                                                                    \
  TEST_P(manifold##JacobiansTester, TEST_##manifold##_LMINUS_JACOBIANS)                                                   \
  { evalLminusJac(); }                                                                                                    \
  TEST_P(manifold##JacobiansTester, TEST_##manifold##_MINUS_JACOBIANS)                                                    \
  { evalMinusJac(); }                                                                                                     \
  TEST_P(manifold##JacobiansTester, TEST_##manifold##_COMPOSE_JACOBIANS)                                                  \
  { evalComposeJac(); }                                                                                                   \
  TEST_P(manifold##JacobiansTester, TEST_##manifold##_BETWEEN_JACOBIANS)                                                  \
  { evalBetweenJac(); }                                                                                                   \
  TEST_P(manifold##JacobiansTester, TEST_##manifold##_ADJ)                                                                \
  { evalAdj(); }                                                                                                          \
  TEST_P(manifold##JacobiansTester, TEST_##manifold##_ADJ_JL_JR)                                                          \
  { evalAdjJlJr(); }                                                                                                      \
  TEST_P(manifold##JacobiansTester, TEST_##manifold##_JLJLinv_JRJRinv)                                                    \
  { evalJrJrinvJlJlinv(); }                                                                                               \
  TEST_P(manifold##JacobiansTester, TEST_##manifold##_ACT_JACOBIANS)                                                      \
  { evalActJac(); }                                                                                                       \
  TEST_P(manifold##JacobiansTester, TEST_##manifold##_PLUS_T_JACOBIANS)                                                   \
  { evalTanPlusTanJac(); }                                                                                                \
  TEST_P(manifold##JacobiansTester, TEST_##manifold##_MINUS_T_JACOBIANS)                                                  \
  { evalTanMinusTanJac(); }                                                                                               \
  TEST_P(manifold##JacobiansTester, TEST_##manifold##_JL_MJLINV_ADJ_JACOBIANS)                                            \
  { evalJlmJlinvAjd(); }                                                                                                  \
  TEST_P(manifold##JacobiansTester, TEST_##manifold##_JR_JR_COMP_JL_JACOBIANS)                                            \
  { evalJrJrcompJl(); }



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
class CommonTester
  : public testing::TestWithParam<
    std::tuple<
      _LieGroup, _LieGroup, typename _LieGroup::Tangent, typename _LieGroup::Tangent
    >
  >
// : public ::testing::Test
{
  using LieGroup = _LieGroup;
  using Scalar   = typename LieGroup::Scalar;
  using Tangent  = typename LieGroup::Tangent;

  const LieGroup& getState() const {
    return std::get<0>(this->GetParam());
  }

  const LieGroup& getStateOther() const {
    return std::get<1>(this->GetParam());
  }

  const Tangent& getDelta() const {
    return std::get<2>(this->GetParam());
  }

  const Tangent& getDeltaOther() const {
    return std::get<3>(this->GetParam());
  }

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
    EXPECT_MANIF_NEAR(getState().rplus(getDelta()),
                      getState().plus(getDelta()), tol_);
  }

  void evalPlusOpIsRplus()
  {
    EXPECT_MANIF_NEAR(getState().rplus(getDelta()),
                      getState() + getDelta(), tol_);
  }

  void evalMinusIsRminus()
  {
    EXPECT_MANIF_NEAR(getState().rminus(getStateOther()),
                      getState().minus(getStateOther()), tol_);
  }

  void evalMinusOpIsRminus()
  {
    EXPECT_MANIF_NEAR(getState().rminus(getStateOther()),
                      getState() - getStateOther(), tol_);
  }

  void evalPlusEq()
  {
    LieGroup ret = getState();
    ret += getDelta();

    EXPECT_MANIF_NEAR(getState() + getDelta(), ret, tol_);
  }

  void evalCompOp()
  {
    EXPECT_MANIF_NEAR(getState().compose(getStateOther()),
                      getState() * getStateOther(), tol_);
  }

  void evalCompEq()
  {
    LieGroup ret = getState();
    ret *= getStateOther();

    EXPECT_MANIF_NEAR(getState() * getStateOther(), ret, tol_);
  }

  void evalLiftRetr()
  {
    EXPECT_MANIF_NEAR(getState(), getState().log().exp(), tol_);
  }

  void evalRetrLift()
  {
    EXPECT_MANIF_NEAR(getDelta(), getDelta().exp().log(), tol_);
  }

  void evalComposeWithInv()
  {
    EXPECT_MANIF_NEAR(LieGroup::Identity(),
                      getState().compose(getState().inverse()), tol_);
    EXPECT_MANIF_NEAR(LieGroup::Identity(),
                      getState() * getState().inverse(), tol_);

    LieGroup tmp = getState(); tmp *= getState().inverse();
    EXPECT_MANIF_NEAR(LieGroup::Identity(), tmp, tol_);

    EXPECT_MANIF_NEAR(getState(), getState() * LieGroup::Identity(), tol_);
    EXPECT_MANIF_NEAR(getState(), LieGroup::Identity() * getState(), tol_);
  }

  void evalBetweenSelf()
  {
    EXPECT_MANIF_NEAR(LieGroup::Identity(),
                      getState().between(getState()), tol_);
  }

  void evalPlusZero()
  {
    EXPECT_MANIF_NEAR(getState(),
                      getState() + Tangent::Zero(), tol_);
  }

  void evalMinusSelf()
  {
    EXPECT_MANIF_NEAR(Tangent::Zero(),
                      getState() - getState(), tol_);
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

    EXPECT_THROW(average_biinvariant(std::vector<LieGroup, Eigen::aligned_allocator<LieGroup>>{}),
                 std::runtime_error);

    {
      const auto dummy = LieGroup::Random();
      std::vector<LieGroup, Eigen::aligned_allocator<LieGroup>> tmp;
      tmp.push_back(dummy);
      EXPECT_MANIF_NEAR(dummy, average_biinvariant(tmp), tol_);
    }

    const LieGroup centroid = LieGroup::Random();

    std::vector<LieGroup, Eigen::aligned_allocator<LieGroup>> mans;

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

      EXPECT_MANIF_NEAR(avg, avg_shu, (std::is_same<Scalar, float>::value)? 1e-2 : 1e-4);
    }
  }

  void evalIsApprox()
  {
    // Group

    EXPECT_TRUE(LieGroup::Identity().isApprox(LieGroup::Identity(), tol_));

    EXPECT_TRUE(LieGroup::Identity() == LieGroup::Identity());

    EXPECT_TRUE(getState().isApprox(getState(), tol_));
    EXPECT_FALSE(getState().isApprox(LieGroup::Random(), tol_));

    // cppcheck-suppress duplicateExpression
    EXPECT_TRUE(getState() == getState());
    EXPECT_FALSE(getState() == LieGroup::Random());

    // Tangent

    EXPECT_TRUE(Tangent::Zero().isApprox(Tangent::Zero(), tol_));

    EXPECT_TRUE(Tangent::Zero() == Tangent::Zero());

    EXPECT_TRUE(getDelta().isApprox(getDelta(), tol_));
    EXPECT_FALSE(getDelta().isApprox(Tangent::Random(), tol_));

    // cppcheck-suppress duplicateExpression
    EXPECT_TRUE(getDelta() == getDelta());
    EXPECT_FALSE(getDelta() == Tangent::Random());
  }

  void evalUnaryMinus()
  {
    Tangent minus_delta = -getDelta();
    typename Tangent::DataType delta_data = getDelta().coeffs();
    typename Tangent::DataType minus_delta_data = minus_delta.coeffs();
    EXPECT_EIGEN_NEAR(-delta_data, minus_delta_data);
    EXPECT_EIGEN_NEAR(-getDelta().coeffs(), minus_delta.coeffs());
  }

  void evalSomeTangentOperators()
  {
    typename Tangent::DataType delta_data(getDelta().coeffs());

    // t+t
    EXPECT_EIGEN_NEAR(getDelta().plus(getDelta()).coeffs(), delta_data+delta_data);

    // t-t
    EXPECT_EIGEN_NEAR(getDelta().minus(getDelta()).coeffs(), delta_data-delta_data);

    // t+t
    EXPECT_EIGEN_NEAR((getDelta()+getDelta()).coeffs(), delta_data+delta_data);

    // t-t
    EXPECT_EIGEN_NEAR((getDelta()-getDelta()).coeffs(), delta_data-delta_data);

    // t+v
    EXPECT_EIGEN_NEAR((getDelta()+delta_data).coeffs(), delta_data+delta_data);

    // t-v
    EXPECT_EIGEN_NEAR((getDelta()-delta_data).coeffs(), delta_data-delta_data);

    // v+t
    EXPECT_EIGEN_NEAR(delta_data+getDelta(), delta_data+delta_data);

    // v-t
    EXPECT_EIGEN_NEAR(delta_data-getDelta(), delta_data-delta_data);

    // ret type is Tangent
    EXPECT_EIGEN_NEAR(
      (getDelta()+getDelta()+getDelta()-getDelta()+delta_data-delta_data+getDelta()/2.+getDelta()*3.).coeffs(),
      delta_data+delta_data+delta_data-delta_data+delta_data-delta_data+delta_data/2.+delta_data*3.
    );

    // ret type is Tangent::DataType
    EXPECT_EIGEN_NEAR(
      delta_data+getDelta()+getDelta()-getDelta()+delta_data-delta_data+getDelta()/2.+getDelta()*3.,
      delta_data+delta_data+delta_data-delta_data+delta_data-delta_data+delta_data/2.+delta_data*3.
    );

    Tangent w;
    w << delta_data;

    EXPECT_EIGEN_NEAR(delta_data, w.coeffs());

    EXPECT_EIGEN_NEAR((getDelta()*3.14).coeffs(), delta_data*3.14);
    EXPECT_EIGEN_NEAR((3.14*getDelta()).coeffs(), 3.14*delta_data);

    EXPECT_EIGEN_NEAR((getDelta()/5.12).coeffs(), delta_data/5.12);
  }

  void evalSomeTangentOperators2()
  {
    Tangent delta_copy = getDelta();
    typename Tangent::DataType delta_data(delta_copy.coeffs());

    // t+=t
    EXPECT_EIGEN_NEAR((delta_copy+=delta_copy).coeffs(), delta_data+=delta_data);

    // t-=t
    EXPECT_EIGEN_NEAR((delta_copy-=delta_copy).coeffs(), delta_data-=delta_data);

    // t+=v
    EXPECT_EIGEN_NEAR((delta_copy+=delta_data).coeffs(), delta_data+=delta_data);

    // t-=v
    EXPECT_EIGEN_NEAR((delta_copy-=delta_data).coeffs(), delta_data-=delta_data);

    Tangent w;
    w << delta_data;

    EXPECT_EIGEN_NEAR((delta_copy*=3.14).coeffs(), delta_data*=3.14);

    EXPECT_EIGEN_NEAR((delta_copy/=5.12).coeffs(), delta_data/=5.12);
  }

  void evalGeneratorsHat()
  {
    EXPECT_THROW(Tangent::Generator(-1), manif::invalid_argument);
    EXPECT_THROW(Tangent::Generator(42), manif::invalid_argument);

    typename Tangent::LieAlg sum_delta_hat;
    sum_delta_hat.setZero();

    for (int i=0; i<Tangent::DoF; ++i)
    {
      sum_delta_hat += getDelta().coeffs()(i) * Tangent::Generator(i);
    }

    EXPECT_EIGEN_NEAR(getDelta().hat(), sum_delta_hat);

    sum_delta_hat.setZero();
    for (int i=0; i<Tangent::DoF; ++i)
    {
      sum_delta_hat += getDelta().coeffs()(i) * getDelta().generator(i);
    }

    EXPECT_EIGEN_NEAR(getDelta().hat(), sum_delta_hat);
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

  void evalInnerZero()
  {
    EXPECT_DOUBLE_EQ(0., Tangent::Zero().weightedNorm());
    EXPECT_DOUBLE_EQ(0., Tangent::Zero().squaredWeightedNorm());
    EXPECT_DOUBLE_EQ(0., Tangent::Zero().inner(Tangent::Zero()));
  }

  void evalInner()
  {
    EXPECT_DOUBLE_EQ(
      getDelta().squaredWeightedNorm(), getDelta().inner(getDelta())
    );
    EXPECT_DOUBLE_EQ(
      getDelta().inner(getDeltaOther()), getDeltaOther().inner(getDelta())
    );
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
    EXPECT_EIGEN_NEAR(
      (getDelta().smallAdj() * getDeltaOther()).hat(),
      getDelta().hat() * getDeltaOther().hat() - getDeltaOther().hat() * getDelta().hat()
    );
  }

  void evalIdentityActPoint()
  {
    using Point = typename LieGroup::Vector;

    Point pin = Point::Random();

    Point pout = LieGroup::Identity().act(pin);

    EXPECT_EIGEN_NEAR(pin, pout);
  }

  void evalCast() {
    using NewScalar = typename std::conditional<
      std::is_same<Scalar, float>::value, double, float
    >::type;

    EXPECT_NO_THROW(
      auto state = getState().template cast<NewScalar>();
    );

    int i=0;
    EXPECT_NO_THROW(
      for (; i < 10000; ++i) {
        auto state = LieGroup::Random().template cast<NewScalar>();
      }
    ) << "+= failed at iteration " << i;
  }

  void evalInverse() {
    EXPECT_MANIF_NEAR(
      LieGroup::Identity(), LieGroup::Identity()*LieGroup::Identity().inverse()
    );
    EXPECT_MANIF_NEAR(
      LieGroup::Identity(), LieGroup::Identity().inverse()*LieGroup::Identity()
    );
    EXPECT_MANIF_NEAR(LieGroup::Identity(), getState()*getState().inverse());
    EXPECT_MANIF_NEAR(LieGroup::Identity(), getState().inverse()*getState());
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
class JacobianTester
  : public testing::TestWithParam<
    std::tuple<
      _LieGroup, _LieGroup, typename _LieGroup::Tangent, typename _LieGroup::Tangent
    >
  >
{
  using LieGroup = _LieGroup;
  using Scalar   = typename LieGroup::Scalar;
  using Tangent  = typename LieGroup::Tangent;
  using Jacobian = typename LieGroup::Jacobian;

  const LieGroup& getState() const {
    return std::get<0>(this->GetParam());
  }

  const LieGroup& getStateOther() const {
    return std::get<1>(this->GetParam());
  }

  const Tangent& getDelta() const {
    return std::get<2>(this->GetParam());
  }

  const Tangent& getDeltaOther() const {
    return std::get<3>(this->GetParam());
  }

public:

  MANIF_MAKE_ALIGNED_OPERATOR_NEW_COND_TYPE(LieGroup)

  JacobianTester()  = default;
  ~JacobianTester() = default;

  void SetUp() override
  {
    std::srand((unsigned int) time(0));
    w = Tangent(Tangent::DataType::Random()*w_order_);
  }

  void evalInverseJac()
  {
    const LieGroup& state = getState();

    LieGroup state_out = state.inverse(J_out_lhs);

    LieGroup state_pert = (state+w).inverse();
    LieGroup state_lin  = state_out.rplus(J_out_lhs*w);

    EXPECT_MANIF_NEAR(state_pert, state_lin, tol_);
  }

  void evalLiftJac()
  {
    const LieGroup& state = getState();

    Tangent state_out = state.log(J_out_lhs);

    Tangent state_pert = (state+w).log();
    Tangent state_lin  = state_out + (J_out_lhs*w);

    EXPECT_MANIF_NEAR(state_pert, state_lin, tol_);
  }

  void evalRetractJac()
  {
    const Tangent& delta = getDelta();

    LieGroup state_out = delta.exp(J_out_lhs);

    LieGroup state_pert = (delta+w).exp();
    LieGroup state_lin  = state_out + (J_out_lhs*w);

    EXPECT_MANIF_NEAR(state_pert, state_lin, tol_);
  }

  void evalComposeJac()
  {
    const LieGroup& state = getState();
    const LieGroup& state_other = getStateOther();

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
    const LieGroup& state = getState();
    const LieGroup& state_other = getStateOther();

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
    const LieGroup& state = getState();
    const Tangent& delta = getDelta();

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
    const LieGroup& state = getState();
    const Tangent& delta = getDelta();

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
    const LieGroup& state = getState();
    const Tangent& delta = getDelta();

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
    const LieGroup& state = getState();
    const LieGroup& state_other = getStateOther();

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
    const LieGroup& state = getState();
    const LieGroup& state_other = getStateOther();

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
    const LieGroup& state = getState();
    const LieGroup& state_other = getStateOther();

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
    const LieGroup& state = getState();
    const LieGroup& state_other = getStateOther();

    const Tangent& delta = getDelta();

    Jacobian Adja, Adjb, Adjc;

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
    const LieGroup& state = getState();

    Jacobian Adj, Jr, Jl;

    Adj = state.adj();

    Tangent tan = state.log();

    Jr = tan.rjac();
    Jl = tan.ljac();

    EXPECT_EIGEN_NEAR(Jl, Adj*Jr);
    EXPECT_EIGEN_NEAR(Adj, Jl*Jr.inverse());

    // Jr(-tau) = Jl(tau)

    EXPECT_EIGEN_NEAR(Jl, (-tan).rjac());
  }

  void evalJrJrinvJlJlinv()
  {
    Jacobian Jr, Jrinv, Jl, Jlinv;

    const LieGroup& state = getState();

    const Tangent tan = state.log();

    Jr = tan.rjac();
    Jl = tan.ljac();

    Jrinv = tan.rjacinv();
    Jlinv = tan.ljacinv();

    EXPECT_EIGEN_NEAR(Jacobian::Identity(), Jr*Jrinv, tol_);
    EXPECT_EIGEN_NEAR(Jacobian::Identity(), Jl*Jlinv, tol_);
  }

  void evalActJac()
  {
    using Scalar = typename LieGroup::Scalar;
    using Point = typename LieGroup::Vector;
    Point point = Point::Random();

    Eigen::Matrix<Scalar, Point::SizeAtCompileTime, LieGroup::DoF> J_pout_s;
    Eigen::Matrix<Scalar, Point::SizeAtCompileTime, Point::SizeAtCompileTime> J_pout_p;

    const LieGroup& state = getState();

    const Point pointout = state.act(point, J_pout_s, J_pout_p);

    const Point w_point = Point::Random()*w_order_;

    // Jac wrt first element

    Point point_pert = (state+w).act(point);
    Point point_lin  = pointout + (J_pout_s*w.coeffs());

    Scalar tol = (std::is_same<Scalar, float>::value)? 5e-4 : 1e-6;
    EXPECT_EIGEN_NEAR(point_pert, point_lin, tol);

    // Jac wrt second element

    point_pert = state.act(point+w_point);
    point_lin  = pointout + J_pout_p*w_point;

    EXPECT_EIGEN_NEAR(point_pert, point_lin, tol_);
  }

  void evalTanPlusTanJac()
  {
    const Tangent& delta = getDelta();
    const Tangent& delta_other = getDeltaOther();

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
    const Tangent& delta = getDelta();
    const Tangent& delta_other = getDeltaOther();

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

  void evalJlmJlinvAjd() {
    const Tangent& delta = getDelta();
    const Tangent mdelta = -delta;

    EXPECT_EIGEN_NEAR(delta.exp().adj(), delta.ljac()*mdelta.ljacinv());
  }

  void evalJrJrcompJl() {
    const Tangent& delta = getDelta();

    EXPECT_EIGEN_NEAR(delta.exp().inverse().adj() * delta.ljac(), (-delta).ljac());
  }

  void setOmegaOrder(const double w_order) { w_order_ = w_order; }
  void setTolerance(const double tol) { tol_ = tol; }

  double getOmegaOrder() const noexcept { return w_order_; }
  double getTolerance() const noexcept { return tol_; }

protected:

  // relax tolerance for float type
  Scalar w_order_ = (std::is_same<Scalar, float>::value)? 1e-2 : 1e-4;
  Scalar tol_ = (std::is_same<Scalar, float>::value)? 1e-4 : 1e-8;

  Tangent w;

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
