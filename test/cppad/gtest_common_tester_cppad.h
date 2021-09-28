#ifndef _MANIF_MANIF_TEST_CPPAD_COMMON_TESTER_CPPAD_H_
#define _MANIF_MANIF_TEST_CPPAD_COMMON_TESTER_CPPAD_H_

#include "../gtest_manif_utils.h"

#define CPPAD_REVERSE 1

#define MANIF_TEST_CPPAD(manifold)                                                              \
  using TEST_##manifold##_JACOBIANS_CPPAD_TESTER = CommonTesterCppAD<manifold>;                 \
  INSTANTIATE_TEST_CASE_P(                                                                      \
    TEST_##manifold##_JACOBIANS_CPPAD_TESTS,                                                    \
    TEST_##manifold##_JACOBIANS_CPPAD_TESTER,                                                   \
    ::testing::Values(                                                                          \
      std::make_tuple(                                                                          \
        manifold::Identity(),                                                                   \
        manifold::Identity(),                                                                   \
        manifold::Tangent::Zero(),                                                              \
        manifold::Tangent::Zero()                                                               \
      ),                                                                                        \
      std::make_tuple(                                                                          \
        (manifold::Tangent::Random()*1e-8).exp(),                                               \
        (manifold::Tangent::Random()*1e-8).exp(),                                               \
        manifold::Tangent::Random()*1e-8,                                                       \
        manifold::Tangent::Random()*1e-8                                                        \
      ),                                                                                        \
      std::make_tuple(                                                                          \
        manifold::Random(),                                                                     \
        manifold::Random(),                                                                     \
        manifold::Tangent::Random(),                                                            \
        manifold::Tangent::Random()                                                             \
      )                                                                                         \
    ));                                                                                         \
  TEST_P(TEST_##manifold##_JACOBIANS_CPPAD_TESTER, TEST_##manifold##_CPPAD_RMINUS_JACOBIAN)     \
  { evalRminusJac(); }                                                                          \
  TEST_P(TEST_##manifold##_JACOBIANS_CPPAD_TESTER, TEST_##manifold##_CPPAD_LMINUS_JACOBIAN)     \
  { evalLminusJac(); }                                                                          \
  TEST_P(TEST_##manifold##_JACOBIANS_CPPAD_TESTER, TEST_##manifold##_CPPAD_RPLUS_JACOBIAN)      \
  { evalRplusJac(); }                                                                           \
  TEST_P(TEST_##manifold##_JACOBIANS_CPPAD_TESTER, TEST_##manifold##_CPPAD_LPLUS_JACOBIAN)      \
  { evalLplusJac(); }                                                                           \
  TEST_P(TEST_##manifold##_JACOBIANS_CPPAD_TESTER, TEST_##manifold##_CPPAD_LOG_JACOBIAN)        \
  { evalLogJac(); }                                                                             \
  TEST_P(TEST_##manifold##_JACOBIANS_CPPAD_TESTER, TEST_##manifold##_CPPAD_EXP_JACOBIAN)        \
  { evalExpJac(); }                                                                             \
  TEST_P(TEST_##manifold##_JACOBIANS_CPPAD_TESTER, TEST_##manifold##_CPPAD_INVERSE_JACOBIAN)    \
  { evalInverseJac(); }                                                                         \
  TEST_P(TEST_##manifold##_JACOBIANS_CPPAD_TESTER, TEST_##manifold##_CPPAD_COMPOSE_JACOBIAN)    \
  { evalComposeJac(); }                                                                         \
  TEST_P(TEST_##manifold##_JACOBIANS_CPPAD_TESTER, TEST_##manifold##_CPPAD_BETWEEN_JACOBIAN)    \
  { evalBetweenJac(); }                                                                         \
  TEST_P(TEST_##manifold##_JACOBIANS_CPPAD_TESTER, TEST_##manifold##_CPPAD_ACT_JACOBIAN)        \
  { evalActJac(); }                                                                             \
  TEST_P(TEST_##manifold##_JACOBIANS_CPPAD_TESTER, TEST_##manifold##_CPPAD_TAPE_JACOBIAN)       \
  { evalTape(); }

#define __MANIF_MAKE_TEST_CPPAD(manifold, type) \
  using manifold##type = manifold<type>;        \
  MANIF_TEST_CPPAD(manifold##type)

#define __MANIF_MAKE_TEST_CPPAD_ALL_TYPES(manifold) \
  __MANIF_MAKE_TEST_CPPAD(manifold, double)

  // float-based test are a little too flaky ~[1e-2, 1e-6]
  // __MANIF_MAKE_TEST_CPPAD(manifold, float)

#define MANIF_TEST_CPPAD_ALL                \
  using namespace manif;                    \
  __MANIF_MAKE_TEST_CPPAD_ALL_TYPES(R2)     \
  __MANIF_MAKE_TEST_CPPAD_ALL_TYPES(R5)     \
  __MANIF_MAKE_TEST_CPPAD_ALL_TYPES(SO2)    \
  __MANIF_MAKE_TEST_CPPAD_ALL_TYPES(SE2)    \
  __MANIF_MAKE_TEST_CPPAD_ALL_TYPES(SO3)    \
  __MANIF_MAKE_TEST_CPPAD_ALL_TYPES(SE3)    \
  __MANIF_MAKE_TEST_CPPAD_ALL_TYPES(SE_2_3)


template <typename _LieGroup>
class CppADLocalReParameterizationFunctor {
  using LieGroup = _LieGroup;
  using Scalar   = typename _LieGroup::Scalar;
  using Tangent  = typename _LieGroup::Tangent;

  template <typename _Scalar>
  using LieGroupTemplate = typename LieGroup::template LieGroupTemplate<_Scalar>;

  template <typename _Scalar>
  using TangentTemplate = typename Tangent::template TangentTemplate<_Scalar>;

  using Ad = CppAD::AD<Scalar>;

  using Vs = Eigen::Matrix<Scalar, LieGroup::DoF, 1>;

public:

  enum class Mode : char {Aut, For, Rev};

  using MatrixXad = Eigen::Matrix<Ad, Eigen::Dynamic, 1>;
  using Jacobian = Eigen::Matrix<
    Scalar, LieGroup::DoF, LieGroup::RepSize, Eigen::RowMajor
  >;

  CppADLocalReParameterizationFunctor() = default;
  virtual ~CppADLocalReParameterizationFunctor() = default;

  template <typename T>
  bool operator()(
    T* state_raw, T* state_other_raw, T* delta_raw
  ) const {
    const Eigen::Map<LieGroupTemplate<T>> state(state_raw);
    const Eigen::Map<LieGroupTemplate<T>> state_other(state_other_raw);
    Eigen::Map<TangentTemplate<T>>  delta(delta_raw);

    delta = state - state_other;

    return true;
  }

  template <Mode M = Mode::Aut>
  Jacobian ComputeJacobian(
    Eigen::Ref<Eigen::Matrix<Scalar, LieGroup::RepSize, 1>> _state
  ) {
    MatrixXad state = _state.template cast<Ad>();
    MatrixXad state_other = state;
    MatrixXad delta(LieGroup::DoF);

    CppAD::Independent(state);

    this->template operator()<Ad>(state.data(), state_other.data(), delta.data());

    CppAD::ADFun<Scalar> ad_fun(state, delta);

    MANIF_ASSERT(delta.isZero(manif::Constants<Ad>::eps));

    Eigen::Matrix<Scalar, Eigen::Dynamic, 1> jac;

    switch (M) {
    case Mode::Aut:
      jac = ad_fun.Jacobian(state.template cast<Scalar>().eval());
      break;
    case Mode::For:
      jac.resize(ad_fun.Domain()*ad_fun.Range());
      CppAD::JacobianFor(ad_fun, state.template cast<Scalar>().eval(), jac);
      break;
    case Mode::Rev:
      jac.resize(ad_fun.Domain()*ad_fun.Range());
      CppAD::JacobianRev(ad_fun, state.template cast<Scalar>().eval(), jac);
      break;
    default:
      MANIF_THROW("Unknown auto differentiation mode.");
      break;
    }

    MANIF_ASSERT(LieGroup::RepSize * LieGroup::DoF == jac.size());

    return Eigen::Map<Jacobian>(jac.data());
  }
};

template <typename _LieGroup>
class CommonTesterCppAD : public testing::TestWithParam<std::tuple<
  _LieGroup, _LieGroup, typename _LieGroup::Tangent, typename _LieGroup::Tangent
>> {
  using LieGroup  = _LieGroup;
  using Scalar    = typename LieGroup::Scalar;
  using Tangent   = typename LieGroup::Tangent;
  using Jacobian  = typename LieGroup::Jacobian;

  template <typename _Scalar>
  using LieGroupTemplate = typename LieGroup::template LieGroupTemplate<_Scalar>;

  template <typename _Scalar>
  using TangentTemplate = typename Tangent::template TangentTemplate<_Scalar>;

  using Ad = CppAD::AD<Scalar>;
  using AdFun = CppAD::ADFun<Scalar>;

  using LocalReParameterization = CppADLocalReParameterizationFunctor<LieGroup>;

  using LPJacobian = Eigen::Matrix<
    Scalar,
    LieGroup::RepSize,
    LieGroup::DoF,
    (LieGroup::DoF>1)? Eigen::RowMajor : Eigen::ColMajor
  >;

  using VectorXs  = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;
  using VectorXad = Eigen::Matrix<Ad, Eigen::Dynamic, 1>;

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

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  CommonTesterCppAD()  = default;
  ~CommonTesterCppAD() = default;

  void evalRminusJac() {
    // The variable block.
    // @note CppAD doesn't like fixed size vector,
    // we thus work with a Eigen dynamic one...
    VectorXad variables(LieGroup::RepSize + LieGroup::RepSize);

    VectorXad delta_vec(Tangent::RepSize);

    Eigen::Map<LieGroupTemplate<Ad>>
      state(variables.data()),
      state_other(variables.data()+LieGroup::RepSize);
    Eigen::Map<TangentTemplate<Ad>> delta(delta_vec.data());

    state = getState().template cast<Ad>();
    state_other = getStateOther().template cast<Ad>();

    // declare independent variables and start taping
    CppAD::Independent(variables);

    delta = state.rminus(state_other);

    // create f: state, state_other -> delta
    // and stop taping
    AdFun ad_fun(variables, delta_vec);

    // Compute analytic Jacobians
    Jacobian J_d_s, J_d_so;
    EXPECT_MANIF_NEAR(
      getState().rminus(getStateOther(), J_d_s, J_d_so),
      delta.template cast<Scalar>(),
      tol_
    );

    VectorXs variables_s = variables.template cast<Scalar>().eval();

    // Compute AD Jacobians at this state, state_other

    // Forward
    {
      VectorXs jac(ad_fun.Domain()*ad_fun.Range());
      CppAD::JacobianFor(ad_fun, variables_s, jac);

      EXPECT_EQ(LieGroup::DoF * LieGroup::RepSize * 2, jac.size());

      // Compute the local parameterization Jacobian
      // that maps to the local tangent space.
      LPJacobian lpJ_s_0 = cppadLocalParameterizationJacobian(
        getState(), manif::AutoDifferentiation::For
      );

      LPJacobian lpJ_so_0 = cppadLocalParameterizationJacobian(
        getStateOther(), manif::AutoDifferentiation::For
      );

      // Retrieve the respective state, state_other block Jacobians
      // @note CppAD seem to have a RowMajor layout
      Eigen::Map<Eigen::Matrix<
        Scalar, LieGroup::DoF, LieGroup::RepSize*2, Eigen::RowMajor
      >> adJ(jac.data());

      // Compare Analytic vs AD.
      EXPECT_EIGEN_NEAR(
        J_d_s, adJ.block(0, 0, LieGroup::DoF, LieGroup::RepSize) * lpJ_s_0, tol_
      );
      EXPECT_EIGEN_NEAR(
        J_d_so,
        adJ.block(0, LieGroup::RepSize, LieGroup::DoF, LieGroup::RepSize) * lpJ_so_0,
        tol_
      );
    }

    // Reverse
    if (CPPAD_REVERSE) {
      VectorXs jac(ad_fun.Domain()*ad_fun.Range());
      CppAD::JacobianRev(ad_fun, variables_s, jac);

      EXPECT_EQ(LieGroup::DoF * LieGroup::RepSize * 2, jac.size());

      // Compute the local parameterization Jacobian
      // that maps to the local tangent space.
      LPJacobian lpJ_s_0 = cppadLocalParameterizationJacobian(
        getState(), manif::AutoDifferentiation::Rev
      );

      LPJacobian lpJ_so_0 = cppadLocalParameterizationJacobian(
        getStateOther(), manif::AutoDifferentiation::Rev
      );

      // Retrieve the respective state, state_other block Jacobians
      // @note CppAD seem to have a RowMajor layout
      Eigen::Map<Eigen::Matrix<
        Scalar, LieGroup::DoF, LieGroup::RepSize*2, Eigen::RowMajor
      >> adJ(jac.data());

      // Compare Analytic vs AD.
      EXPECT_EIGEN_NEAR(
        J_d_s, adJ.block(0, 0, LieGroup::DoF, LieGroup::RepSize) * lpJ_s_0, tol_
      );
      EXPECT_EIGEN_NEAR(
        J_d_so,
        adJ.block(0, LieGroup::RepSize, LieGroup::DoF, LieGroup::RepSize) * lpJ_so_0,
        tol_
      );
    }
  }

  void evalLminusJac() {
    // The variable block.
    // @note CppAD doesn't like fixed size vector,
    // we thus work with a Eigen dynamic one...
    VectorXad variables(LieGroup::RepSize + LieGroup::RepSize);

    VectorXad delta_vec(Tangent::RepSize);

    Eigen::Map<LieGroupTemplate<Ad>>
      state(variables.data()),
      state_other(variables.data()+LieGroup::RepSize);
    Eigen::Map<TangentTemplate<Ad>> delta(delta_vec.data());

    state = getState().template cast<Ad>();
    state_other = getStateOther().template cast<Ad>();

    // declare independent variables and start taping
    CppAD::Independent(variables);

    delta = state.lminus(state_other);

    // create f: state, state_other -> delta
    // and stop taping
    AdFun ad_fun(variables, delta_vec);

    // Compute analytic Jacobians
    Jacobian J_d_s, J_d_so;
    EXPECT_MANIF_NEAR(
      getState().lminus(getStateOther(), J_d_s, J_d_so),
      delta.template cast<Scalar>(),
      tol_
    );

    VectorXs variables_s = variables.template cast<Scalar>().eval();

    // Compute AD Jacobians at this state, state_other

    // Forward
    {
      VectorXs jac(ad_fun.Domain()*ad_fun.Range());
      CppAD::JacobianFor(ad_fun, variables_s, jac);

      EXPECT_EQ(LieGroup::DoF * LieGroup::RepSize * 2, jac.size());

      // Compute the local parameterization Jacobian
      // that maps to the local tangent space.
      LPJacobian lpJ_s_0 = cppadLocalParameterizationJacobian(
        getState(), manif::AutoDifferentiation::For
      );

      LPJacobian lpJ_so_0 = cppadLocalParameterizationJacobian(
        getStateOther(), manif::AutoDifferentiation::For
      );

      // Retrieve the respective state, state_other block Jacobians
      // @note CppAD seem to have a RowMajor layout
      Eigen::Map<Eigen::Matrix<
        Scalar, LieGroup::DoF, LieGroup::RepSize*2, Eigen::RowMajor
      >> adJ(jac.data());

      // Compare Analytic vs AD.
      EXPECT_EIGEN_NEAR(
        J_d_s, adJ.block(0, 0, LieGroup::DoF, LieGroup::RepSize) * lpJ_s_0, tol_
      );
      EXPECT_EIGEN_NEAR(
        J_d_so,
        adJ.block(0, LieGroup::RepSize, LieGroup::DoF, LieGroup::RepSize) * lpJ_so_0,
        tol_
      );
    }

    // Reverse
    if (CPPAD_REVERSE) {
      VectorXs jac(ad_fun.Domain()*ad_fun.Range());
      CppAD::JacobianRev(ad_fun, variables_s, jac);

      EXPECT_EQ(LieGroup::DoF * LieGroup::RepSize * 2, jac.size());

      // Compute the local parameterization Jacobian
      // that maps to the local tangent space.
      LPJacobian lpJ_s_0 = cppadLocalParameterizationJacobian(
        getState(), manif::AutoDifferentiation::Rev
      );

      LPJacobian lpJ_so_0 = cppadLocalParameterizationJacobian(
        getStateOther(), manif::AutoDifferentiation::Rev
      );

      // Retrieve the respective state, state_other block Jacobians
      // @note CppAD seem to have a RowMajor layout
      Eigen::Map<Eigen::Matrix<
        Scalar, LieGroup::DoF, LieGroup::RepSize*2, Eigen::RowMajor
      >> adJ(jac.data());

      // Compare Analytic vs AD.
      EXPECT_EIGEN_NEAR(
        J_d_s, adJ.block(0, 0, LieGroup::DoF, LieGroup::RepSize) * lpJ_s_0, tol_
      );
      EXPECT_EIGEN_NEAR(
        J_d_so,
        adJ.block(0, LieGroup::RepSize, LieGroup::DoF, LieGroup::RepSize) * lpJ_so_0,
        tol_
      );
    }
  }

  void evalRplusJac() {
    VectorXad variables(LieGroup::RepSize + LieGroup::DoF);

    VectorXad variables_out(LieGroup::RepSize);

    Eigen::Map<LieGroupTemplate<Ad>> state(variables.data());
    Eigen::Map<TangentTemplate<Ad>> delta(variables.data()+LieGroup::RepSize);
    Eigen::Map<LieGroupTemplate<Ad>> state_out(variables_out.data());

    state = getState().template cast<Ad>();
    delta = getDelta().template cast<Ad>();

    // declare independent variables and start taping
    CppAD::Independent(variables);

    state_out = state.rplus(delta);

    // create f: state, delta -> state_out
    // and stop taping
    AdFun ad_fun(variables, variables_out);

    // Compute analytic Jacobians
    Jacobian J_so_s, J_so_d;
    EXPECT_MANIF_NEAR(
      getState().rplus(getDelta(), J_so_s, J_so_d),
      state_out.template cast<Scalar>(),
      tol_
    );

    VectorXs variables_s = variables.template cast<Scalar>().eval();

    // Compute AD Jacobians at this state, state_other

    // Forward
    {
      VectorXs jac(ad_fun.Domain()*ad_fun.Range());
      CppAD::JacobianFor(ad_fun, variables_s, jac);

      EXPECT_EQ((LieGroup::RepSize + LieGroup::DoF) * LieGroup::RepSize, jac.size());

      // Compute the local parameterization Jacobian
      // that maps to the local tangent space.
      LPJacobian lpJ_s_0 = cppadLocalParameterizationJacobian(
        getState(), manif::AutoDifferentiation::For
      );

      // Compute the local parameterization Jacobian
      // that maps to the local tangent space.

      VectorXs variables_out_s = variables_out.template cast<Scalar>();

      typename LocalReParameterization::Jacobian lrpJ = LocalReParameterization(
      ).template ComputeJacobian<LocalReParameterization::Mode::For>(variables_out_s);

      // Retrieve the respective state, state_other block Jacobians
      // @note CppAD seem to have a RowMajor layout
      Eigen::Map<Eigen::Matrix<
        Scalar, LieGroup::RepSize, LieGroup::RepSize + LieGroup::DoF, Eigen::RowMajor
      >> adJ(jac.data());

      // Compare Analytic vs AD.
      EXPECT_EIGEN_NEAR(
        J_so_s,
        lrpJ * adJ.block(0, 0, LieGroup::RepSize, LieGroup::RepSize) * lpJ_s_0,
        tol_
      );
      EXPECT_EIGEN_NEAR(
        J_so_d,
        lrpJ * adJ.block(0, LieGroup::RepSize, LieGroup::RepSize, LieGroup::DoF),
        tol_
      );
    }

    // Reverse
    if (CPPAD_REVERSE) {
      VectorXs jac(ad_fun.Domain()*ad_fun.Range());
      CppAD::JacobianRev(ad_fun, variables_s, jac);

      EXPECT_EQ((LieGroup::RepSize + LieGroup::DoF) * LieGroup::RepSize, jac.size());

      // Compute the local parameterization Jacobian
      // that maps to the local tangent space.
      LPJacobian lpJ_s_0 = cppadLocalParameterizationJacobian(
        getState(), manif::AutoDifferentiation::Rev
      );

      // Compute the local parameterization Jacobian
      // that maps to the local tangent space.

      VectorXs variables_out_s = variables_out.template cast<Scalar>();

      typename LocalReParameterization::Jacobian lrpJ = LocalReParameterization(
      ).template ComputeJacobian<LocalReParameterization::Mode::Rev>(variables_out_s);

      // Retrieve the respective state, state_other block Jacobians
      // @note CppAD seem to have a RowMajor layout
      Eigen::Map<Eigen::Matrix<
        Scalar, LieGroup::RepSize, LieGroup::RepSize + LieGroup::DoF, Eigen::RowMajor
      >> adJ(jac.data());

      // Compare Analytic vs AD.
      EXPECT_EIGEN_NEAR(
        J_so_s,
        lrpJ * adJ.block(0, 0, LieGroup::RepSize, LieGroup::RepSize) * lpJ_s_0,
        tol_
      );
      EXPECT_EIGEN_NEAR(
        J_so_d,
        lrpJ * adJ.block(0, LieGroup::RepSize, LieGroup::RepSize, LieGroup::DoF),
        tol_
      );
    }
  }

  void evalLplusJac() {
    VectorXad variables(LieGroup::RepSize + LieGroup::DoF);

    VectorXad variables_out(LieGroup::RepSize);

    Eigen::Map<LieGroupTemplate<Ad>> state(variables.data());
    Eigen::Map<TangentTemplate<Ad>> delta(variables.data()+LieGroup::RepSize);
    Eigen::Map<LieGroupTemplate<Ad>> state_out(variables_out.data());

    state = getState().template cast<Ad>();
    delta = getDelta().template cast<Ad>();

    // declare independent variables and start taping
    CppAD::Independent(variables);

    state_out = state.lplus(delta);

    // create f: state, delta -> state_out
    // and stop taping
    AdFun ad_fun(variables, variables_out);

    // Compute analytic Jacobians
    Jacobian J_so_s, J_so_d;
    EXPECT_MANIF_NEAR(
      getState().lplus(getDelta(), J_so_s, J_so_d),
      state_out.template cast<Scalar>(),
      tol_
    );

    VectorXs variables_s = variables.template cast<Scalar>().eval();

    // Compute AD Jacobians at this state, state_other

    // Forward
    {
      VectorXs jac(ad_fun.Domain()*ad_fun.Range());
      CppAD::JacobianFor(ad_fun, variables_s, jac);

      EXPECT_EQ((LieGroup::RepSize + LieGroup::DoF) * LieGroup::RepSize, jac.size());

      // Compute the local parameterization Jacobian
      // that maps to the local tangent space.
      LPJacobian lpJ_s_0 = cppadLocalParameterizationJacobian(
        getState(), manif::AutoDifferentiation::For
      );

      // Compute the local parameterization Jacobian
      // that maps to the local tangent space.

      VectorXs variables_out_s = variables_out.template cast<Scalar>();

      typename LocalReParameterization::Jacobian lrpJ = LocalReParameterization(
      ).template ComputeJacobian<LocalReParameterization::Mode::For>(variables_out_s);

      // Retrieve the respective state, state_other block Jacobians
      // @note CppAD seem to have a RowMajor layout
      Eigen::Map<Eigen::Matrix<
        Scalar, LieGroup::RepSize, LieGroup::RepSize + LieGroup::DoF, Eigen::RowMajor
      >> adJ(jac.data());

      // Compare Analytic vs AD.
      EXPECT_EIGEN_NEAR(
        J_so_s,
        lrpJ * adJ.block(0, 0, LieGroup::RepSize, LieGroup::RepSize) * lpJ_s_0,
        tol_
      );
      EXPECT_EIGEN_NEAR(
        J_so_d,
        lrpJ * adJ.block(0, LieGroup::RepSize, LieGroup::RepSize, LieGroup::DoF),
        tol_
      );
    }

    // Reverse
    if (CPPAD_REVERSE) {
      VectorXs jac(ad_fun.Domain()*ad_fun.Range());
      CppAD::JacobianRev(ad_fun, variables_s, jac);

      EXPECT_EQ((LieGroup::RepSize + LieGroup::DoF) * LieGroup::RepSize, jac.size());

      // Compute the local parameterization Jacobian
      // that maps to the local tangent space.
      LPJacobian lpJ_s_0 = cppadLocalParameterizationJacobian(
        getState(), manif::AutoDifferentiation::Rev
      );

      // Compute the local parameterization Jacobian
      // that maps to the local tangent space.

      VectorXs variables_out_s = variables_out.template cast<Scalar>();

      typename LocalReParameterization::Jacobian lrpJ = LocalReParameterization(
      ).template ComputeJacobian<LocalReParameterization::Mode::Rev>(variables_out_s);

      // Retrieve the respective state, state_other block Jacobians
      // @note CppAD seem to have a RowMajor layout
      Eigen::Map<Eigen::Matrix<
        Scalar, LieGroup::RepSize, LieGroup::RepSize + LieGroup::DoF, Eigen::RowMajor
      >> adJ(jac.data());

      // Compare Analytic vs AD.
      EXPECT_EIGEN_NEAR(
        J_so_s,
        lrpJ * adJ.block(0, 0, LieGroup::RepSize, LieGroup::RepSize) * lpJ_s_0,
        tol_
      );
      EXPECT_EIGEN_NEAR(
        J_so_d,
        lrpJ * adJ.block(0, LieGroup::RepSize, LieGroup::RepSize, LieGroup::DoF),
        tol_
      );
    }
  }

  void evalLogJac() {
    VectorXad variables(LieGroup::RepSize);
    VectorXad delta_vec(Tangent::RepSize);

    Eigen::Map<LieGroupTemplate<Ad>> state(variables.data());
    Eigen::Map<TangentTemplate<Ad>> delta(delta_vec.data());

    state = getState().template cast<Ad>();

    // declare independent variables and start taping
    CppAD::Independent(variables);

    delta = state.log();

    // create f: state -> delta and stop taping
    AdFun ad_fun(variables, delta_vec);

    // Compute analytic Jacobians
    Jacobian J_d_s;
    EXPECT_MANIF_NEAR(getState().log(J_d_s), delta.template cast<Scalar>(), tol_);

    auto variables_s = variables.template cast<Scalar>().eval();

    // Compute AD Jacobians at this state

    // Forward
    {
      VectorXs jac(ad_fun.Domain()*ad_fun.Range());
      CppAD::JacobianFor(ad_fun, variables_s, jac);

      EXPECT_EQ(LieGroup::DoF * LieGroup::RepSize, jac.size());

      // Compute the local parameterization Jacobian
      // that maps to the local tangent space.
      LPJacobian lpJ = cppadLocalParameterizationJacobian(
        getState(), manif::AutoDifferentiation::For
      );

      // Retrieve the block Jacobians
      Eigen::Map<Eigen::Matrix<
        Scalar, LieGroup::DoF, LieGroup::RepSize, Eigen::RowMajor
      >> adJ_d_s(jac.data());

      // Compare Analytic vs AD.
      EXPECT_EIGEN_NEAR(J_d_s, adJ_d_s * lpJ, tol_);
    }

    // Reverse
    if (CPPAD_REVERSE) {
      VectorXs jac(ad_fun.Domain()*ad_fun.Range());
      CppAD::JacobianRev(ad_fun, variables_s, jac);

      EXPECT_EQ(LieGroup::DoF * LieGroup::RepSize, jac.size());

      // Compute the local parameterization Jacobian
      // that maps to the local tangent space.
      LPJacobian lpJ = cppadLocalParameterizationJacobian(
        getState(), manif::AutoDifferentiation::Rev
      );

      // Retrieve the block Jacobians
      Eigen::Map<Eigen::Matrix<
        Scalar, LieGroup::DoF, LieGroup::RepSize, Eigen::RowMajor
      >> adJ_d_s(jac.data());

      // Compare Analytic vs AD.
      EXPECT_EIGEN_NEAR(J_d_s, adJ_d_s * lpJ, tol_);
    }
  }

  void evalExpJac() {
    VectorXad variables(Tangent::RepSize);
    VectorXad variables_out(LieGroup::RepSize);

    Eigen::Map<TangentTemplate<Ad>> delta(variables.data());
    Eigen::Map<LieGroupTemplate<Ad>> state_out(variables_out.data());

    delta = getDelta().template cast<Ad>();

    // declare independent variables and start taping
    CppAD::Independent(variables);

    state_out = delta.exp();

    // create f: state -> delta and stop taping
    AdFun ad_fun(variables, variables_out);

    // Compute analytic Jacobians
    Jacobian J_x_d;
    EXPECT_MANIF_NEAR(
      getDelta().exp(J_x_d), state_out.template cast<Scalar>(), tol_
    );

    auto variables_s = variables.template cast<Scalar>().eval();

    // Compute AD Jacobians at this state
    {
      VectorXs jac(ad_fun.Domain()*ad_fun.Range());
      CppAD::JacobianFor(ad_fun, variables_s, jac);

      EXPECT_EQ(LieGroup::RepSize * LieGroup::DoF, jac.size());

      // Compute the local parameterization Jacobian
      // that maps to the local tangent space.

      VectorXs variables_out_s = variables_out.template cast<Scalar>();

      typename LocalReParameterization::Jacobian lrpJ = LocalReParameterization(
      ).template ComputeJacobian<LocalReParameterization::Mode::For>(variables_out_s);

      // Retrieve the block Jacobians
      Eigen::Map<Eigen::Matrix<
        Scalar,
        LieGroup::RepSize,
        LieGroup::DoF,
        (LieGroup::DoF>1)? Eigen::RowMajor : Eigen::ColMajor
      >> adJ_x_d(jac.data());

      // Compare Analytic vs AD.
      EXPECT_EIGEN_NEAR(J_x_d, lrpJ * adJ_x_d, tol_);
    }

    // @todo Error detected by false result for
    // size_t(x.size()) == f.Domain()

    if (CPPAD_REVERSE) {
      VectorXs jac(ad_fun.Domain()*ad_fun.Range());
      CppAD::JacobianRev(ad_fun, variables_s, jac);

      EXPECT_EQ(LieGroup::RepSize * LieGroup::DoF, jac.size());

      // Compute the local parameterization Jacobian
      // that maps to the local tangent space.

      VectorXs variables_out_s = variables_out.template cast<Scalar>();

      typename LocalReParameterization::Jacobian lrpJ = LocalReParameterization(
      ).template ComputeJacobian<LocalReParameterization::Mode::Rev>(variables_out_s);

      // Retrieve the block Jacobians
      Eigen::Map<Eigen::Matrix<
        Scalar,
        LieGroup::RepSize,
        LieGroup::DoF,
        (LieGroup::DoF>1)? Eigen::RowMajor : Eigen::ColMajor
      >> adJ_x_d(jac.data());

      // Compare Analytic vs AD.
      EXPECT_EIGEN_NEAR(J_x_d, lrpJ * adJ_x_d, tol_);
    }
  }

  void evalInverseJac() {
    VectorXad variables(LieGroup::RepSize);
    VectorXad state_out_vec(LieGroup::RepSize);

    Eigen::Map<LieGroupTemplate<Ad>> state(variables.data());
    Eigen::Map<LieGroupTemplate<Ad>> state_out(state_out_vec.data());

    state = getState().template cast<Ad>();

    // declare independent variables and start taping
    CppAD::Independent(variables);

    state_out = state.inverse();

    // create f: state -> delta and stop taping
    AdFun ad_fun(variables, state_out_vec);

    // Compute analytic Jacobians
    Jacobian J_d_s;
    EXPECT_MANIF_NEAR(
      getState().inverse(J_d_s), state_out.template cast<Scalar>(), tol_
    );

    // Compute AD Jacobians at this state
    {
      auto variables_s = variables.template cast<Scalar>().eval();

      VectorXs jac(ad_fun.Domain()*ad_fun.Range());
      CppAD::JacobianFor(ad_fun, variables_s, jac);

      EXPECT_EQ(LieGroup::RepSize * LieGroup::RepSize, jac.size());

      // Compute the local parameterization Jacobian
      // that maps to the local tangent space.
      LPJacobian lpJ = cppadLocalParameterizationJacobian(
        getState(), manif::AutoDifferentiation::For
      );

      variables_s = state_out_vec.template cast<Scalar>();

      typename LocalReParameterization::Jacobian lrpJ = LocalReParameterization(
      ).template ComputeJacobian<LocalReParameterization::Mode::For>(variables_s);

      // Retrieve the block Jacobians
      Eigen::Map<Eigen::Matrix<
        Scalar, LieGroup::RepSize, LieGroup::RepSize, Eigen::RowMajor
      >> adJ_d_s(jac.data());

      // Compare Analytic vs AD.
      EXPECT_EIGEN_NEAR(J_d_s, lrpJ * adJ_d_s * lpJ, tol_);
    }

    if (CPPAD_REVERSE) {
      auto variables_s = variables.template cast<Scalar>().eval();

      VectorXs jac(ad_fun.Domain()*ad_fun.Range());
      CppAD::JacobianRev(ad_fun, variables_s, jac);

      EXPECT_EQ(LieGroup::RepSize * LieGroup::RepSize, jac.size());

      // Compute the local parameterization Jacobian
      // that maps to the local tangent space.
      LPJacobian lpJ = cppadLocalParameterizationJacobian(
        getState(), manif::AutoDifferentiation::Rev
      );

      variables_s = state_out_vec.template cast<Scalar>();

      typename LocalReParameterization::Jacobian lrpJ = LocalReParameterization(
      ).template ComputeJacobian<LocalReParameterization::Mode::Rev>(variables_s);

      // Retrieve the block Jacobians
      Eigen::Map<Eigen::Matrix<
        Scalar, LieGroup::RepSize, LieGroup::RepSize, Eigen::RowMajor
      >> adJ_d_s(jac.data());

      // Compare Analytic vs AD.
      EXPECT_EIGEN_NEAR(J_d_s, lrpJ * adJ_d_s * lpJ, tol_);
    }
  }

  void evalComposeJac() {
    VectorXad variables(LieGroup::RepSize + LieGroup::RepSize);

    VectorXad variables_out(LieGroup::RepSize);

    Eigen::Map<LieGroupTemplate<Ad>>
      state(variables.data()),
      state_other(variables.data()+LieGroup::RepSize),
      state_out(variables_out.data());

    state = getState().template cast<Ad>();
    state_other = getStateOther().template cast<Ad>();

    // declare independent variables and start taping
    CppAD::Independent(variables);

    state_out = state.compose(state_other);

    // create f: state, state_other -> delta
    // and stop taping
    AdFun ad_fun(variables, variables_out);

    // Compute analytic Jacobians
    Jacobian J_cs_s, J_cs_so;
    EXPECT_MANIF_NEAR(
      getState().compose(getStateOther(), J_cs_s, J_cs_so),
      state_out.template cast<Scalar>(),
      tol_
    );

    VectorXs variables_s = variables.template cast<Scalar>().eval();

    // Compute AD Jacobians at this state, state_other

    // Forward
    {
      VectorXs jac(ad_fun.Domain()*ad_fun.Range());
      CppAD::JacobianFor(ad_fun, variables_s, jac);

      EXPECT_EQ(LieGroup::RepSize * LieGroup::RepSize * 2, jac.size());

      // Compute the local parameterization Jacobian
      // that maps to the local tangent space.
      LPJacobian lpJ_s_0 = cppadLocalParameterizationJacobian(
        getState(), manif::AutoDifferentiation::For
      );

      LPJacobian lpJ_so_0 = cppadLocalParameterizationJacobian(
        getStateOther(), manif::AutoDifferentiation::For
      );

      VectorXs variables_out_s = variables_out.template cast<Scalar>();

      typename LocalReParameterization::Jacobian lrpJ = LocalReParameterization(
      ).template ComputeJacobian<LocalReParameterization::Mode::For>(variables_out_s);

      // Retrieve the respective state, state_other block Jacobians
      // @note CppAD seem to have a RowMajor layout
      Eigen::Map<Eigen::Matrix<
        Scalar, LieGroup::RepSize, LieGroup::RepSize*2, Eigen::RowMajor
      >> adJ(jac.data());

      // Compare Analytic vs AD.
      EXPECT_EIGEN_NEAR(
        J_cs_s,
        lrpJ * adJ.block(0, 0, LieGroup::RepSize, LieGroup::RepSize) * lpJ_s_0,
        tol_
      );
      EXPECT_EIGEN_NEAR(
        J_cs_so,
        lrpJ * adJ.block(0, LieGroup::RepSize, LieGroup::RepSize, LieGroup::RepSize) * lpJ_so_0,
        tol_
      );
    }

    // Reverse
    if (CPPAD_REVERSE) {
      VectorXs jac(ad_fun.Domain()*ad_fun.Range());
      CppAD::JacobianRev(ad_fun, variables_s, jac);

      EXPECT_EQ(LieGroup::RepSize * LieGroup::RepSize * 2, jac.size());

      // Compute the local parameterization Jacobian
      // that maps to the local tangent space.
      LPJacobian lpJ_s_0 = cppadLocalParameterizationJacobian(
        getState(), manif::AutoDifferentiation::Rev
      );

      LPJacobian lpJ_so_0 = cppadLocalParameterizationJacobian(
        getStateOther(), manif::AutoDifferentiation::Rev
      );

      VectorXs variables_out_s = variables_out.template cast<Scalar>();

      typename LocalReParameterization::Jacobian lrpJ = LocalReParameterization(
      ).template ComputeJacobian<LocalReParameterization::Mode::Rev>(variables_out_s);

      // Retrieve the respective state, state_other block Jacobians
      // @note CppAD seem to have a RowMajor layout
      Eigen::Map<Eigen::Matrix<
        Scalar, LieGroup::RepSize, LieGroup::RepSize*2, Eigen::RowMajor
      >> adJ(jac.data());

      // Compare Analytic vs AD.
      EXPECT_EIGEN_NEAR(
        J_cs_s,
        lrpJ * adJ.block(0, 0, LieGroup::RepSize, LieGroup::RepSize) * lpJ_s_0,
        tol_
      );
      EXPECT_EIGEN_NEAR(
        J_cs_so,
        lrpJ * adJ.block(0, LieGroup::RepSize, LieGroup::RepSize, LieGroup::RepSize) * lpJ_so_0,
        tol_
      );
    }
  }

  void evalBetweenJac() {
    VectorXad variables(LieGroup::RepSize + LieGroup::RepSize);

    VectorXad variables_out(LieGroup::RepSize);

    Eigen::Map<LieGroupTemplate<Ad>>
      state(variables.data()),
      state_other(variables.data()+LieGroup::RepSize),
      state_out(variables_out.data());

    state = getState().template cast<Ad>();
    state_other = getStateOther().template cast<Ad>();

    // declare independent variables and start taping
    CppAD::Independent(variables);

    state_out = state.between(state_other);

    // create f: state, state_other -> delta
    // and stop taping
    AdFun ad_fun(variables, variables_out);

    // Compute analytic Jacobians
    Jacobian J_cs_s, J_cs_so;
    EXPECT_MANIF_NEAR(
      getState().between(getStateOther(), J_cs_s, J_cs_so),
      state_out.template cast<Scalar>(),
      tol_
    );

    VectorXs variables_s = variables.template cast<Scalar>().eval();

    // Compute AD Jacobians at this state, state_other

    // Forward
    {
      VectorXs jac(ad_fun.Domain()*ad_fun.Range());
      CppAD::JacobianFor(ad_fun, variables_s, jac);

      EXPECT_EQ(LieGroup::RepSize * LieGroup::RepSize * 2, jac.size());

      // Compute the local parameterization Jacobian
      // that maps to the local tangent space.
      LPJacobian lpJ_s_0 = cppadLocalParameterizationJacobian(
        getState(), manif::AutoDifferentiation::For
      );

      LPJacobian lpJ_so_0 = cppadLocalParameterizationJacobian(
        getStateOther(), manif::AutoDifferentiation::For
      );

      VectorXs variables_out_s = variables_out.template cast<Scalar>();

      typename LocalReParameterization::Jacobian lrpJ = LocalReParameterization(
      ).template ComputeJacobian<LocalReParameterization::Mode::For>(variables_out_s);

      // Retrieve the respective state, state_other block Jacobians
      // @note CppAD seem to have a RowMajor layout
      Eigen::Map<Eigen::Matrix<
        Scalar, LieGroup::RepSize, LieGroup::RepSize*2, Eigen::RowMajor
      >> adJ(jac.data());

      // Compare Analytic vs AD.
      EXPECT_EIGEN_NEAR(
        J_cs_s,
        lrpJ * adJ.block(0, 0, LieGroup::RepSize, LieGroup::RepSize) * lpJ_s_0,
        tol_
      );
      EXPECT_EIGEN_NEAR(
        J_cs_so,
        lrpJ * adJ.block(0, LieGroup::RepSize, LieGroup::RepSize, LieGroup::RepSize) * lpJ_so_0,
        tol_
      );
    }

    // Reverse
    if (CPPAD_REVERSE) {
      VectorXs jac(ad_fun.Domain()*ad_fun.Range());
      CppAD::JacobianRev(ad_fun, variables_s, jac);

      EXPECT_EQ(LieGroup::RepSize * LieGroup::RepSize * 2, jac.size());

      // Compute the local parameterization Jacobian
      // that maps to the local tangent space.
      LPJacobian lpJ_s_0 = cppadLocalParameterizationJacobian(
        getState(), manif::AutoDifferentiation::Rev
      );

      LPJacobian lpJ_so_0 = cppadLocalParameterizationJacobian(
        getStateOther(), manif::AutoDifferentiation::Rev
      );

      VectorXs variables_out_s = variables_out.template cast<Scalar>();

      typename LocalReParameterization::Jacobian lrpJ = LocalReParameterization(
      ).template ComputeJacobian<LocalReParameterization::Mode::Rev>(variables_out_s);

      // Retrieve the respective state, state_other block Jacobians
      // @note CppAD seem to have a RowMajor layout
      Eigen::Map<Eigen::Matrix<
        Scalar, LieGroup::RepSize, LieGroup::RepSize*2, Eigen::RowMajor
      >> adJ(jac.data());

      // Compare Analytic vs AD.
      EXPECT_EIGEN_NEAR(
        J_cs_s,
        lrpJ * adJ.block(0, 0, LieGroup::RepSize, LieGroup::RepSize) * lpJ_s_0,
        tol_
      );
      EXPECT_EIGEN_NEAR(
        J_cs_so,
        lrpJ * adJ.block(0, LieGroup::RepSize, LieGroup::RepSize, LieGroup::RepSize) * lpJ_so_0,
        tol_
      );
    }
  }

  void evalActJac() {
    VectorXad variables(LieGroup::RepSize + LieGroup::Dim);

    VectorXad variables_out(LieGroup::Dim);

    Eigen::Matrix<Scalar, LieGroup::Dim, 1> vector_s;
    vector_s.setRandom();

    Eigen::Map<LieGroupTemplate<Ad>> state(variables.data());
    Eigen::Map<Eigen::Matrix<Ad, LieGroup::Dim, 1>>
      vector(variables.data()+LieGroup::RepSize),
      vector_out(variables_out.data());

    state = getState().template cast<Ad>();
    vector = vector_s.template cast<Ad>();

    // declare independent variables and start taping
    CppAD::Independent(variables);

    vector_out = state.act(vector);

    // create f: state, vec -> vec_out
    // and stop taping
    AdFun ad_fun(variables, variables_out);

    // Compute analytic Jacobians
    Eigen::Matrix<Scalar, LieGroup::Dim, LieGroup::DoF> J_cs_s;
    Eigen::Matrix<Scalar, LieGroup::Dim, LieGroup::Dim> J_cs_so;
    EXPECT_EIGEN_NEAR(
      getState().act(vector_s, J_cs_s, J_cs_so),
      vector_out.template cast<Scalar>(),
      tol_
    );

    VectorXs variables_s = variables.template cast<Scalar>().eval();

    // Compute AD Jacobians at this state, state_other

    // Forward
    {
      VectorXs jac(ad_fun.Domain()*ad_fun.Range());
      CppAD::JacobianFor(ad_fun, variables_s, jac);

      EXPECT_EQ((LieGroup::RepSize + LieGroup::Dim) * LieGroup::Dim, jac.size());

      // Compute the local parameterization Jacobian
      // that maps to the local tangent space.
      LPJacobian lpJ_s_0 = cppadLocalParameterizationJacobian(
        getState(), manif::AutoDifferentiation::For
      );

      // Retrieve the respective state, state_other block Jacobians
      // @note CppAD seem to have a RowMajor layout
      Eigen::Map<Eigen::Matrix<
        Scalar, LieGroup::Dim, LieGroup::RepSize + LieGroup::Dim, Eigen::RowMajor
      >> adJ(jac.data());

      // Compare Analytic vs AD.
      EXPECT_EIGEN_NEAR(
        J_cs_s, adJ.block(0, 0, LieGroup::Dim, LieGroup::RepSize) * lpJ_s_0, tol_
      );
      EXPECT_EIGEN_NEAR(
        J_cs_so, adJ.block(0, LieGroup::RepSize, LieGroup::Dim, LieGroup::Dim), tol_
      );
    }

    // Reverse
    if (CPPAD_REVERSE) {
      VectorXs jac(ad_fun.Domain()*ad_fun.Range());
      CppAD::JacobianRev(ad_fun, variables_s, jac);

      EXPECT_EQ((LieGroup::RepSize + LieGroup::Dim) * LieGroup::Dim, jac.size());

      // Compute the local parameterization Jacobian
      // that maps to the local tangent space.
      LPJacobian lpJ_s_0 = cppadLocalParameterizationJacobian(
        getState(), manif::AutoDifferentiation::Rev
      );

      // Retrieve the respective state, state_other block Jacobians
      // @note CppAD seem to have a RowMajor layout
      Eigen::Map<Eigen::Matrix<
        Scalar, LieGroup::Dim, LieGroup::RepSize + LieGroup::Dim, Eigen::RowMajor
      >> adJ(jac.data());

      // Compare Analytic vs AD.
      EXPECT_EIGEN_NEAR(
        J_cs_s, adJ.block(0, 0, LieGroup::Dim, LieGroup::RepSize) * lpJ_s_0, tol_
      );
      EXPECT_EIGEN_NEAR(
        J_cs_so, adJ.block(0, LieGroup::RepSize, LieGroup::Dim, LieGroup::Dim), tol_
      );
    }
  }

  void evalTape() {
    // We evaluate here the jacs of a dummy, complex expression
    // to make sure that cppad's tape holds on.

    LieGroup sstate = getState();
    LieGroup sstate_other = getStateOther();

    VectorXad variables(LieGroup::RepSize + LieGroup::RepSize);

    VectorXad delta_vec(Tangent::RepSize);

    Eigen::Map<LieGroupTemplate<Ad>>
      state(variables.data()),
      state_other(variables.data()+LieGroup::RepSize);
    Eigen::Map<TangentTemplate<Ad>> delta(delta_vec.data());

    state = sstate.template cast<Ad>();
    state_other = sstate_other.template cast<Ad>();

    // declare independent variables and start taping
    CppAD::Independent(variables);

    delta = state.rminus(state).exp().rminus(state_other).exp().log();

    // create f: state, state_other -> delta
    // and stop taping
    AdFun ad_fun(variables, delta_vec);

    // Compute analytic Jacobians ds/dd, dso/dd
    Jacobian J_I_slhs, J_I_srhs, J_exp_I, J_rm_exp, J_rm_so, J_exprm_rm, J_d_exprm;
    EXPECT_MANIF_NEAR(
      sstate.rminus(
        sstate, J_I_slhs, J_I_srhs
      ).exp(J_exp_I).rminus(
        sstate_other, J_rm_exp, J_rm_so
      ).exp(J_exprm_rm).log(J_d_exprm),
      delta.template cast<Scalar>(),
      tol_
    );

    Jacobian J_d_rm = J_d_exprm * J_exprm_rm;
    Jacobian J_d_I = J_d_rm * J_rm_exp * J_exp_I;
    Jacobian J_d_s = J_d_I * J_I_slhs + J_d_I * J_I_srhs;
    Jacobian J_d_so = J_d_rm * J_rm_so;

    VectorXs variables_s = variables.template cast<Scalar>().eval();

    // Compute AD Jacobians at this state, state_other

    // Forward
    {
      VectorXs jac(ad_fun.Domain()*ad_fun.Range());
      CppAD::JacobianFor(ad_fun, variables_s, jac);

      EXPECT_EQ(LieGroup::DoF * LieGroup::RepSize * 2, jac.size());

      // Compute the local parameterization Jacobian
      // that maps to the local tangent space.
      LPJacobian lpJ_s_0 = cppadLocalParameterizationJacobian(
        getState(), manif::AutoDifferentiation::For
      );

      LPJacobian lpJ_so_0 = cppadLocalParameterizationJacobian(
        getStateOther(), manif::AutoDifferentiation::For
      );

      // Retrieve the respective state, state_other block Jacobians
      // @note CppAD seem to have a RowMajor layout
      Eigen::Map<Eigen::Matrix<
        Scalar, LieGroup::DoF, LieGroup::RepSize*2, Eigen::RowMajor
      >> adJ(jac.data());

      // Compare Analytic vs AD.
      EXPECT_EIGEN_NEAR(
        J_d_s, adJ.block(0, 0, LieGroup::DoF, LieGroup::RepSize) * lpJ_s_0, tol_
      );
      EXPECT_EIGEN_NEAR(
        J_d_so,
        adJ.block(0, LieGroup::RepSize, LieGroup::DoF, LieGroup::RepSize) * lpJ_so_0,
        tol_
      );
    }

    // Reverse
    {
      VectorXs jac(ad_fun.Domain()*ad_fun.Range());
      CppAD::JacobianRev(ad_fun, variables_s, jac);

      EXPECT_EQ(LieGroup::DoF * LieGroup::RepSize * 2, jac.size());

      // Compute the local parameterization Jacobian
      // that maps to the local tangent space.
      LPJacobian lpJ_s_0 = cppadLocalParameterizationJacobian(
        getState(), manif::AutoDifferentiation::Rev
      );

      LPJacobian lpJ_so_0 = cppadLocalParameterizationJacobian(
        getStateOther(), manif::AutoDifferentiation::Rev
      );

      // Retrieve the respective state, state_other block Jacobians
      // @note CppAD seem to have a RowMajor layout
      Eigen::Map<Eigen::Matrix<
        Scalar, LieGroup::DoF, LieGroup::RepSize*2, Eigen::RowMajor
      >> adJ(jac.data());

      // Compare Analytic vs AD.
      EXPECT_EIGEN_NEAR(
        J_d_s, adJ.block(0, 0, LieGroup::DoF, LieGroup::RepSize) * lpJ_s_0, tol_
      );
      EXPECT_EIGEN_NEAR(
        J_d_so,
        adJ.block(0, LieGroup::RepSize, LieGroup::DoF, LieGroup::RepSize) * lpJ_so_0,
        tol_
      );
    }
  }

protected:

  Scalar tol_ = (std::is_same<Scalar, float>::value)? 1e-3 : 1e-8;
};

#endif // _MANIF_MANIF_TEST_CPPAD_COMMON_TESTER_CPPAD_H_