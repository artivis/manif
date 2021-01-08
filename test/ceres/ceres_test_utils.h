#ifndef _MANIF_MANIF_CERES_TEST_UTILS_H_
#define _MANIF_MANIF_CERES_TEST_UTILS_H_

#include "../test_utils.h"
#include "manif/ceres/ceres.h"

#define MANIF_TEST_JACOBIANS_CERES(manifold)                                                    \
  using TEST_##manifold##_JACOBIANS_CERES_TESTER = JacobianCeresTester<manifold>;               \
  TEST_F(TEST_##manifold##_JACOBIANS_CERES_TESTER, TEST_##manifold##_CERES_INVERSE_JACOBIAN)    \
  { evalInverseJac(); }                                                                         \
  TEST_F(TEST_##manifold##_JACOBIANS_CERES_TESTER, TEST_##manifold##_CERES_LOG_JACOBIAN)        \
  { evalLogJac(); }                                                                             \
  TEST_F(TEST_##manifold##_JACOBIANS_CERES_TESTER, TEST_##manifold##_CERES_RMINUS_JACOBIANS)    \
  { evalRminusJacs(); }                                                                         \
  TEST_F(TEST_##manifold##_JACOBIANS_CERES_TESTER, TEST_##manifold##_CERES_LMINUS_JACOBIANS)    \
  { evalLminusJacs(); }                                                                         \
  TEST_F(TEST_##manifold##_JACOBIANS_CERES_TESTER, TEST_##manifold##_CERES_COMPOSE_JACOBIANS)   \
  { evalComposeJacs(); }                                                                        \
  TEST_F(TEST_##manifold##_JACOBIANS_CERES_TESTER, TEST_##manifold##_CERES_BETWEEN_JACOBIANS)   \
  { evalBetweenJacs(); }

#define __MANIF_FUNCTOR_COMMON_TYPEDEF                                            \
  using LieGroup = _LieGroup;                                                     \
  using Tangent  = typename _LieGroup::Tangent;                                   \
                                                                                  \
  template <typename _Scalar>                                                     \
  using LieGroupTemplate = typename LieGroup::template LieGroupTemplate<_Scalar>; \
                                                                                  \
  template <typename _Scalar>                                                     \
  using TangentTemplate = typename Tangent::template TangentTemplate<_Scalar>;

namespace manif {

template <
  template <typename LieGroup> class Derived,
  unsigned int O, unsigned int I, typename LieGroup
>
struct MakeUnaryCostFuncHelper
{
  using Jacobian = Eigen::Matrix<
    typename LieGroup::Scalar, O, I, Eigen::RowMajor
  >;

  static
  std::shared_ptr<ceres::AutoDiffCostFunction<Derived<LieGroup>, O, I>>
  make_costfunc()
  {
    return std::make_shared<ceres::AutoDiffCostFunction<Derived<LieGroup>, O, I>>(
      new Derived<LieGroup>()
    );
  }
};

template <
  template <typename _LieGroup> class _Derived,
  unsigned int _O, unsigned int _I0, unsigned int _I1, typename _LieGroup
>
struct MakeBinaryCostFuncHelper
{
  static
  std::shared_ptr<ceres::AutoDiffCostFunction<_Derived<_LieGroup>, _O, _I0, _I1>>
  make_costfunc()
  {
    return std::make_shared<ceres::AutoDiffCostFunction<_Derived<_LieGroup>, _O, _I0, _I1>>(
      new _Derived<_LieGroup>()
    );
  }
};

template <typename _LieGroup>
struct CeresRminusFunctor : MakeBinaryCostFuncHelper<
  CeresRminusFunctor, _LieGroup::DoF, _LieGroup::RepSize, _LieGroup::RepSize, _LieGroup
>
{
  __MANIF_FUNCTOR_COMMON_TYPEDEF;

  using Jacobian =
    Eigen::Matrix<typename LieGroup::Scalar,
                  LieGroup::DoF,
                  LieGroup::RepSize,
                  Eigen::RowMajor>;

  template <typename T>
  bool operator()(const T* const lhs_raw,
                  const T* const rhs_raw,
                  T* residuals_raw) const
  {
    const Eigen::Map<const LieGroupTemplate<T>> lhs_state(lhs_raw);
    const Eigen::Map<const LieGroupTemplate<T>> rhs_state(rhs_raw);

    Eigen::Map<TangentTemplate<T>> residuals(residuals_raw);

    residuals = lhs_state - rhs_state;

    return true;
  }
};

template <typename _LieGroup>
struct CeresLminusFunctor : MakeBinaryCostFuncHelper<
  CeresLminusFunctor, _LieGroup::DoF, _LieGroup::RepSize, _LieGroup::RepSize, _LieGroup
>
{
  __MANIF_FUNCTOR_COMMON_TYPEDEF;

  using Jacobian =
    Eigen::Matrix<typename LieGroup::Scalar,
                  LieGroup::DoF,
                  LieGroup::RepSize,
                  Eigen::RowMajor>;

  template <typename T>
  bool operator()(const T* const lhs_raw,
                  const T* const rhs_raw,
                  T* residuals_raw) const
  {
    const Eigen::Map<const LieGroupTemplate<T>> lhs_state(lhs_raw);
    const Eigen::Map<const LieGroupTemplate<T>> rhs_state(rhs_raw);

    Eigen::Map<TangentTemplate<T>> residuals(residuals_raw);

    residuals = lhs_state.lminus(rhs_state);

    return true;
  }
};

template <typename _LieGroup>
struct CeresInverseFunctor : MakeUnaryCostFuncHelper<
  CeresInverseFunctor, _LieGroup::RepSize, _LieGroup::RepSize, _LieGroup
>
{
  template <typename _Scalar>
  using LieGroupTemplate = typename _LieGroup::template LieGroupTemplate<_Scalar>;

  template <typename T>
  bool operator()(const T* const state_raw,
                  T* residuals_raw) const
  {
    const Eigen::Map<const LieGroupTemplate<T>> state(state_raw);
    Eigen::Map<LieGroupTemplate<T>> residuals(residuals_raw);

    residuals = state.inverse();

    return true;
  }
};

template <typename _LieGroup>
struct CeresLogFunctor : MakeUnaryCostFuncHelper<
  CeresLogFunctor, _LieGroup::DoF, _LieGroup::RepSize, _LieGroup
>
{
  __MANIF_FUNCTOR_COMMON_TYPEDEF;

  template <typename T>
  bool operator()(const T* const state_raw,
                  T* residuals_raw) const
  {
    const Eigen::Map<const LieGroupTemplate<T>> state(state_raw);
    Eigen::Map<TangentTemplate<T>> residuals(residuals_raw);

    residuals = state.log();

    return true;
  }
};

template <typename _LieGroup>
struct CeresComposeFunctor : MakeBinaryCostFuncHelper<
  CeresComposeFunctor, _LieGroup::RepSize, _LieGroup::RepSize, _LieGroup::RepSize, _LieGroup
>
{
  __MANIF_FUNCTOR_COMMON_TYPEDEF;

  using Jacobian =
    Eigen::Matrix<typename LieGroup::Scalar,
                  LieGroup::RepSize,
                  LieGroup::RepSize,
                  Eigen::RowMajor>;

  template <typename T>
  bool operator()(const T* const lhs_raw,
                  const T* const rhs_raw,
                  T* residuals_raw) const
  {
    const Eigen::Map<const LieGroupTemplate<T>> lhs_state(lhs_raw);
    const Eigen::Map<const LieGroupTemplate<T>> rhs_state(rhs_raw);

    Eigen::Map<LieGroupTemplate<T>> residuals(residuals_raw);

    residuals = lhs_state.compose(rhs_state);

    return true;
  }
};

template <typename _LieGroup>
struct CeresBetweenFunctor : MakeBinaryCostFuncHelper<
  CeresBetweenFunctor, _LieGroup::RepSize, _LieGroup::RepSize, _LieGroup::RepSize, _LieGroup
>
{
  __MANIF_FUNCTOR_COMMON_TYPEDEF;

  using Jacobian =
    Eigen::Matrix<typename LieGroup::Scalar,
                  LieGroup::RepSize,
                  LieGroup::RepSize,
                  Eigen::RowMajor>;

  template <typename T>
  bool operator()(const T* const lhs_raw,
                  const T* const rhs_raw,
                  T* residuals_raw) const
  {
    const Eigen::Map<const LieGroupTemplate<T>> lhs_state(lhs_raw);
    const Eigen::Map<const LieGroupTemplate<T>> rhs_state(rhs_raw);

    Eigen::Map<LieGroupTemplate<T>> residuals(residuals_raw);

    residuals = lhs_state.between(rhs_state);

    return true;
  }
};

template <typename _LieGroup>
class JacobianCeresTester : public ::testing::Test
{
  using LieGroup  = _LieGroup;
  using Tangent   = typename LieGroup::Tangent;
  using Jacobian  = typename LieGroup::Jacobian;

  using ManifObjective = CeresObjectiveFunctor<LieGroup>;
  using ManifLocalParameterization = CeresLocalParameterizationFunctor<LieGroup>;

  using LocalParamJacobian =
    Eigen::Matrix<typename LieGroup::Scalar,
                  LieGroup::RepSize,
                  LieGroup::DoF,
                  (LieGroup::DoF>1)?
                    Eigen::RowMajor:
                    Eigen::ColMajor>;

  using LocalParameterizationPtr = std::shared_ptr<ceres::LocalParameterization>;

  using Inverse = CeresInverseFunctor<LieGroup>;
  using Rminus = CeresRminusFunctor<LieGroup>;
  using Lminus = CeresLminusFunctor<LieGroup>;
  using Log = CeresLogFunctor<LieGroup>;
  using Compose = CeresComposeFunctor<LieGroup>;
  using Between = CeresBetweenFunctor<LieGroup>;

  // void local_deparameterization_ComputeJacobian(
  //   double* state,
  //   double* lhs_adJ_locdepar
  // )
  // {
  //   // Compute the local_de-parameterization (?) Jac
  //   double** params = new double*[2];
  //   params[0] = state;
  //   params[1] = state;

  //   typename Rminus::Jacobian rhs_adJ_locdepar;
  //   double** jacs = new double*[2];
  //   jacs[0] = lhs_adJ_locdepar;
  //   jacs[1] = rhs_adJ_locdepar.data();

  //   Tangent d;

  //   Rminus::make_costfunc()->Evaluate(params, d.data(), jacs);

  //   EXPECT_MANIF_NEAR(Tangent::Zero(), d);

  //   delete[] params;
  //   delete[] jacs;
  // }

public:

  MANIF_MAKE_ALIGNED_OPERATOR_NEW_COND_TYPE(LieGroup)

  JacobianCeresTester()  = default;
  ~JacobianCeresTester() = default;

  void SetUp() override
  {
    std::srand((unsigned int) time(0));

    state = LieGroup::Random();
    state_other = LieGroup::Random();
    state_out = LieGroup::Random();

    delta = Tangent::Random();

    state_raw = state.data();
    state_other_raw = state_other.data();
    state_out_raw = state_out.data();

    delta_raw = delta.data();
  }

  void evalInverseJac()
  {
    parameters = &state_raw;

    typename Inverse::Jacobian ad_r_J_out_spd;
    double*  ad_r_J_out_s_raw = ad_r_J_out_spd.data();
    jacobians = &ad_r_J_out_s_raw;

    Inverse::make_costfunc()->Evaluate(parameters, state_other_raw, jacobians);

    EXPECT_MANIF_NEAR(state.inverse(), state_other);

    local_parameterization->ComputeJacobian(state_raw, adJ_locpar.data());

    // Compute the local_de-parameterization (?) Jac
    parameters = new double*[2];
    parameters[0] = state_other_raw;
    parameters[1] = state_other_raw;

    typename Rminus::Jacobian adJ_locdepar, dadJ_out_RO;
    jacobians = new double*[2];
    jacobians[0] = adJ_locdepar.data();
    jacobians[1] = dadJ_out_RO.data();

    Rminus::make_costfunc()->Evaluate(parameters, delta_raw, jacobians);

    EXPECT_MANIF_NEAR(Tangent::Zero(), delta);

    // typename Rminus::Jacobian adJ_locdepar;
    // local_deparameterization_ComputeJacobian(
    //   state_other_raw, adJ_locdepar.data()
    // );

    adJ_out_s = adJ_locdepar * (ad_r_J_out_spd * adJ_locpar);

    state.inverse(J_out_s);

    EXPECT_EIGEN_NEAR(J_out_s, adJ_out_s);

    delete[] parameters;
    delete[] jacobians;
  }

  void evalLogJac()
  {
    parameters = &state_raw;

    typename Log::Jacobian ad_r_J_out_spd;
    double*  ad_r_J_out_s_raw = ad_r_J_out_spd.data();
    jacobians = &ad_r_J_out_s_raw;

    Log::make_costfunc()->Evaluate(parameters, delta_raw, jacobians);

    EXPECT_MANIF_NEAR(state.log(J_out_s), delta);

    local_parameterization->ComputeJacobian(state_raw, adJ_locpar.data());

    adJ_out_s = ad_r_J_out_spd * adJ_locpar;

    EXPECT_EIGEN_NEAR(J_out_s, adJ_out_s);
  }

  /**
   * @brief evalJacs, Compare the manif analytic rminus Jac to
   * these obtain from ceres autodiff.
   */
  void evalRminusJacs()
  {
    parameters = new double*[2];
    parameters[0] = state_raw;
    parameters[1] = state_other_raw;

    typename Rminus::Jacobian adJ_out_R, adJ_out_RO;
    double*  adJ_out_R_raw = adJ_out_R.data();
    double*  adJ_out_RO_raw = adJ_out_RO.data();
    jacobians = new double*[2];
    jacobians[0] = adJ_out_R_raw;
    jacobians[1] = adJ_out_RO_raw;

    // Autodiff

    Rminus::make_costfunc()->Evaluate(parameters, delta_raw, jacobians);

    EXPECT_MANIF_NEAR(state.rminus(state_other), delta);

    local_parameterization->ComputeJacobian(
      state_raw, lhs_adJ_locpar.data()
    );

    adJ_out_s = adJ_out_R * lhs_adJ_locpar;

    local_parameterization->ComputeJacobian(
      state_other_raw, rhs_adJ_locpar.data()
    );

    adJ_out_so = adJ_out_RO * rhs_adJ_locpar;

    state.rminus(state_other, J_out_s, J_out_so);

    EXPECT_EIGEN_NEAR(J_out_s, adJ_out_s);
    EXPECT_EIGEN_NEAR(J_out_so, adJ_out_so);

    delete[] parameters;
    delete[] jacobians;
  }

  /**
   * @brief evalJacs, Compare the manif analytic lminus Jac to
   * these obtain from ceres autodiff.
   */
  void evalLminusJacs()
  {
    parameters = new double*[2];
    parameters[0] = state_raw;
    parameters[1] = state_other_raw;

    typename Lminus::Jacobian adJ_out_R, adJ_out_RO;
    double*  adJ_out_R_raw = adJ_out_R.data();
    double*  adJ_out_RO_raw = adJ_out_RO.data();
    jacobians = new double*[2];
    jacobians[0] = adJ_out_R_raw;
    jacobians[1] = adJ_out_RO_raw;

    Lminus::make_costfunc()->Evaluate(parameters, delta_raw, jacobians);

    EXPECT_MANIF_NEAR(state.lminus(state_other), delta);

    local_parameterization->ComputeJacobian(
      state_raw, lhs_adJ_locpar.data()
    );

    adJ_out_s = adJ_out_R * lhs_adJ_locpar;

    local_parameterization->ComputeJacobian(
      state_other_raw, rhs_adJ_locpar.data()
    );

    adJ_out_so = adJ_out_RO * rhs_adJ_locpar;

    state.lminus(state_other, J_out_s, J_out_so);

    EXPECT_EIGEN_NEAR(J_out_s, adJ_out_s);
    EXPECT_EIGEN_NEAR(J_out_so, adJ_out_so);

    delete[] parameters;
    delete[] jacobians;
  }

  void evalComposeJacs()
  {
    parameters = new double*[2];
    parameters[0] = state_raw;
    parameters[1] = state_other_raw;

    typename Compose::Jacobian adJ_out_R, adJ_out_RO;
    double*  adJ_out_R_raw = adJ_out_R.data();
    double*  adJ_out_RO_raw = adJ_out_RO.data();
    jacobians = new double*[2];
    jacobians[0] = adJ_out_R_raw;
    jacobians[1] = adJ_out_RO_raw;

    Compose::make_costfunc()->Evaluate(parameters, state_out_raw, jacobians);

    EXPECT_MANIF_NEAR(state.compose(state_other, J_out_s, J_out_so), state_out);

    local_parameterization->ComputeJacobian(
      state_raw, lhs_adJ_locpar.data()
    );

    // Compute the local_de-parameterization (?) Jac
    parameters[0] = state_out_raw;
    parameters[1] = state_out_raw;

    typename Rminus::Jacobian adJ_locdepar, adJ_unused;
    jacobians[0] = adJ_locdepar.data();
    jacobians[1] = adJ_unused.data();

    Rminus::make_costfunc()->Evaluate(parameters, delta_raw, jacobians);
    //

    adJ_out_s = adJ_locdepar * adJ_out_R * lhs_adJ_locpar;

    EXPECT_EIGEN_NEAR(J_out_s, adJ_out_s);

    local_parameterization->ComputeJacobian(
      state_other_raw, rhs_adJ_locpar.data()
    );

    adJ_out_so = adJ_locdepar * adJ_out_RO * rhs_adJ_locpar;

    EXPECT_EIGEN_NEAR(J_out_so, adJ_out_so);

    delete[] parameters;
    delete[] jacobians;
  }

  void evalBetweenJacs()
  {
    parameters = new double*[2];
    parameters[0] = state_raw;
    parameters[1] = state_other_raw;

    typename Between::Jacobian adJ_out_R, adJ_out_RO;
    double*  adJ_out_R_raw = adJ_out_R.data();
    double*  adJ_out_RO_raw = adJ_out_RO.data();
    jacobians = new double*[2];
    jacobians[0] = adJ_out_R_raw;
    jacobians[1] = adJ_out_RO_raw;

    Between::make_costfunc()->Evaluate(parameters, state_out_raw, jacobians);

    EXPECT_MANIF_NEAR(state.between(state_other, J_out_s, J_out_so), state_out);

    local_parameterization->ComputeJacobian(
      state_raw, lhs_adJ_locpar.data()
    );

    // Compute the local_de-parameterization (?) Jac
    parameters[0] = state_out_raw;
    parameters[1] = state_out_raw;

    typename Rminus::Jacobian adJ_locdepar, adJ_unused;
    jacobians[0] = adJ_locdepar.data();
    jacobians[1] = adJ_unused.data();

    Rminus::make_costfunc()->Evaluate(parameters, delta_raw, jacobians);
    //

    adJ_out_s = adJ_locdepar * adJ_out_R * lhs_adJ_locpar;

    EXPECT_EIGEN_NEAR(J_out_s, adJ_out_s);

    local_parameterization->ComputeJacobian(
      state_other_raw, rhs_adJ_locpar.data()
    );

    adJ_out_so = adJ_locdepar * adJ_out_RO * rhs_adJ_locpar;

    EXPECT_EIGEN_NEAR(J_out_so, adJ_out_so);

    delete[] parameters;
    delete[] jacobians;
  }

protected:

  double *state_raw, *state_other_raw, *state_out_raw;
  double **parameters;

  double *delta_raw;

  // double *Ja, *Jb;
  double **jacobians;

  LocalParameterizationPtr local_parameterization =
    make_local_parameterization_autodiff<LieGroup>();

  LieGroup state, state_other, state_out;

  Tangent  delta;

  Jacobian J_out_s, J_out_so,
           adJ_out_s, adJ_out_so;

  LocalParamJacobian adJ_locpar, lhs_adJ_locpar, rhs_adJ_locpar;
};

} // namespace manif

#endif // _MANIF_MANIF_CERES_TEST_UTILS_H_
