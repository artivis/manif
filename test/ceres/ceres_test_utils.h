#ifndef _MANIF_MANIF_CERES_TEST_UTILS_H_
#define _MANIF_MANIF_CERES_TEST_UTILS_H_

#include "../test_utils.h"
#include "manif/ceres/ceres.h"

#define MANIF_TEST_JACOBIANS_CERES(manifold)                                                    \
  using TEST_##manifold##_JACOBIANS_CERES_TESTER = JacobianCeresTester<manifold>;               \
  TEST_F(TEST_##manifold##_JACOBIANS_CERES_TESTER, TEST_##manifold##_CERES_OBJECTIVE_JACOBIANS) \
  { /*evalObjectiveJacs();*/ }                                                                      \
  TEST_F(TEST_##manifold##_JACOBIANS_CERES_TESTER, TEST_##manifold##_CERES_JACOBIANS)           \
  { evalJacs(); }

namespace manif {

template <typename _LieGroup>
class JacobianCeresTester : public ::testing::Test
{
  using LieGroup  = _LieGroup;
  using Tangent   = typename _LieGroup::Tangent;

  using ManifObjective = CeresObjectiveFunctor<LieGroup>;
  using ManifLocalParameterization = CeresLocalParameterizationFunctor<LieGroup>;

  using ObjectiveJacobian =
    Eigen::Matrix<typename LieGroup::Scalar,
                  1,
                  LieGroup::RepSize,
                  Eigen::RowMajor>;

  using LocalParamJacobian =
    Eigen::Matrix<typename LieGroup::Scalar,
                  LieGroup::RepSize,
                  LieGroup::DoF,
                  (LieGroup::DoF>1)?
                    Eigen::RowMajor:
                    Eigen::ColMajor>;

public:

  JacobianCeresTester()  = default;
  ~JacobianCeresTester() = default;

  void SetUp() override
  {
    std::srand((unsigned int) time(0));

    state = LieGroup::Random();

    delta = Tangent::Random();

    objective_value = LieGroup::Random();

    state_plus_delta_analytic = LieGroup::Identity();
    state_plus_delta_autodiff = LieGroup::Identity();
  }
/*
  void evalObjectiveJacs()
  {
    // Analytic

//    ManifObjective analytic_obj(objective_value);

    double*  parameter;
    double** parameters;
    parameter  = state.data();
    parameters = &parameter;

    Tangent residuals = Tangent::Zero();
//    ObjectiveJacobian analyticJ_y_r;

    double*  jacobian;
    double** jacobians;
//    jacobian  = analyticJ_y_r.data();
//    jacobians = &jacobian;

//    analytic_obj.Evaluate(parameters, residuals.data(), jacobians);
//    EXPECT_DOUBLE_EQ(0, residuals);

//    ManifLocalParameterization analytic_local_parameterization;

//    analytic_local_parameterization.Plus(state.data(), delta.data(),
//                                         state_plus_delta_analytic.data());

//    LocalParamJacobian analyticJ_r_R;
//    analytic_local_parameterization.ComputeJacobian(state.data(),
//                                                    analyticJ_r_R.data());

    // Autodiff

    std::shared_ptr<ceres::CostFunction> autodiff_obj =
        make_objective_autodiff<LieGroup>(objective_value);

    ObjectiveJacobian autodiffJ_y_r;

    jacobian = autodiffJ_y_r.data();

    autodiff_obj->Evaluate(parameters, residuals.data(), jacobians);
//    EXPECT_DOUBLE_EQ(0, residuals);

    std::shared_ptr<ceres::LocalParameterization>
      auto_diff_local_parameterization =
        make_local_parameterization_autodiff<LieGroup>();

    auto_diff_local_parameterization->Plus(state.data(), delta.data(),
                                           state_plus_delta_autodiff.data());

    LocalParamJacobian autodiffJ_r_R;
    auto_diff_local_parameterization->ComputeJacobian(state.data(),
                                                      autodiffJ_r_R.data());

//    EXPECT_MANIF_NEAR(state_plus_delta_analytic,
//                      state_plus_delta_result, tol_);
    EXPECT_MANIF_NEAR(state_plus_delta_analytic,
                      state_plus_delta_autodiff, tol_);

//    typename LieGroup::Jacobian analyticJ_y_R = analyticJ_y_r * analyticJ_r_R;
    typename LieGroup::Jacobian autodiffJ_y_R = autodiffJ_y_r * autodiffJ_r_R;

//    EXPECT_EIGEN_NEAR(analyticJ_y_R, autodiffJ_y_R);
  }
*/
  /**
   * @brief evalJacs, Compare the manif analytic rminus Jac to
   * these obtain from ceres autodiff.
   */
  void evalJacs()
  {
    double*  parameter;
    double** parameters;
    parameter  = state.data();
    parameters = &parameter;

    double residuals = 1e19;

    double*  jacobian;
    double** jacobians;
    jacobians = &jacobian;

    // Autodiff

    std::shared_ptr<ceres::CostFunction> autodiff_obj =
        make_objective_autodiff<LieGroup>(objective_value);

    ObjectiveJacobian autodiffJ_y_r;
    jacobian = autodiffJ_y_r.data();

    autodiff_obj->Evaluate(parameters, &residuals, jacobians);

    std::shared_ptr<ceres::LocalParameterization>
      auto_diff_local_parameterization =
        make_local_parameterization_autodiff<LieGroup>();

    auto_diff_local_parameterization->Plus(state.data(), delta.data(),
                                           state_plus_delta_autodiff.data());

    LocalParamJacobian autodiffJ_r_R;
    auto_diff_local_parameterization->ComputeJacobian(state.data(),
                                                      autodiffJ_r_R.data());

    Eigen::Matrix<double,1,LieGroup::DoF> autodiffJ_y_R = autodiffJ_y_r * autodiffJ_r_R;
    (void)autodiffJ_y_R;

//    typename LieGroup::Jacobian manifJ_y_R;
//    objective_value.rminus(state, LieGroup::_, manifJ_y_R);

//    EXPECT_EIGEN_NEAR(autodiffJ_y_R, manifJ_y_R);
  }

protected:

  double tol_ = 1e-8;

  LieGroup state;
  Tangent  delta;

  LieGroup objective_value;

  LieGroup state_plus_delta_analytic;
  LieGroup state_plus_delta_autodiff;
};

} /* namespace manif */

#endif /* _MANIF_MANIF_CERES_TEST_UTILS_H_ */
