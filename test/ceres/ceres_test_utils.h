#ifndef _MANIF_MANIF_CERES_TEST_UTILS_H_
#define _MANIF_MANIF_CERES_TEST_UTILS_H_

#include "../test_utils.h"
#include "manif/ceres/ceres.h"

#define MANIF_TEST_JACOBIANS_CERES(manifold)                                                    \
  using TEST_##manifold##_JACOBIANS_CERES_TESTER = JacobianCeresTester<manifold>;               \
  TEST_F(TEST_##manifold##_JACOBIANS_CERES_TESTER, TEST_##manifold##_CERES_OBJECTIVE_JACOBIANS) \
  { evalObjectiveJacs(); }

namespace manif {

template <typename _Manifold>
class JacobianCeresTester : public ::testing::Test
{
  using Manifold  = _Manifold;
  using Tangent   = typename _Manifold::Tangent;

  using ManifObjective = Objective<Manifold>;
  using ManifLocalParameterization = LocalParameterization<Manifold>;

public:

  JacobianCeresTester()  = default;
  ~JacobianCeresTester() = default;

  void SetUp() override
  {
    std::srand((unsigned int) time(0));

    state = Manifold::Random();

    delta = Tangent::Random();

    state_plus_delta_analytic = Manifold::Identity();
    state_plus_delta_autodiff = Manifold::Identity();
  }

  void evalObjectiveJacs()
  {
    // Analytic

    Manifold objective_value = Manifold::Random();

    ManifObjective analytic_obj(objective_value);

    double*  parameter;
    double** parameters;
    parameter  = state.data();
    parameters = &parameter;

    Tangent residuals = Tangent::Zero();
    typename ManifObjective::Jacobian analyticJ_y_r;

    double*  jacobian;
    double** jacobians;
    jacobian  = analyticJ_y_r.data();
    jacobians = &jacobian;

    analytic_obj.Evaluate(parameters, residuals.data(), jacobians);
//    EXPECT_DOUBLE_EQ(0, residuals);

    ManifLocalParameterization analytic_local_parameterization;

    analytic_local_parameterization.Plus(state.data(), delta.data(),
                                         state_plus_delta_analytic.data());

    typename ManifLocalParameterization::Jacobian analyticJ_r_R;
    analytic_local_parameterization.ComputeJacobian(state.data(),
                                                    analyticJ_r_R.data());

    // Autodiff

    std::shared_ptr<ceres::CostFunction> autodiff_obj =
        make_objective_autodiff<Manifold>(objective_value);

    typename ManifObjective::Jacobian autodiffJ_y_r;

    jacobian = autodiffJ_y_r.data();

    autodiff_obj->Evaluate(parameters, residuals.data(), jacobians);
//    EXPECT_DOUBLE_EQ(0, residuals);

    std::shared_ptr<ceres::LocalParameterization>
      auto_diff_local_parameterization =
        make_local_parametrization_autodiff<Manifold>();

    auto_diff_local_parameterization->Plus(state.data(), delta.data(),
                                           state_plus_delta_autodiff.data());

    typename ManifLocalParameterization::Jacobian autodiffJ_r_R;
    auto_diff_local_parameterization->ComputeJacobian(state.data(),
                                                      autodiffJ_r_R.data());

//    EXPECT_MANIF_NEAR(state_plus_delta_analytic,
//                      state_plus_delta_result, tol_);
    EXPECT_MANIF_NEAR(state_plus_delta_analytic,
                      state_plus_delta_autodiff, tol_);

    typename Manifold::Jacobian analyticJ_y_R = analyticJ_y_r * analyticJ_r_R;
    typename Manifold::Jacobian autodiffJ_y_R = autodiffJ_y_r * autodiffJ_r_R;

    EXPECT_EIGEN_NEAR(analyticJ_y_R, autodiffJ_y_R);

    typename Manifold::Jacobian manifJ_y_R;
    objective_value.rminus(state, Manifold::_, manifJ_y_R);

    EXPECT_EIGEN_NEAR(analyticJ_y_R, manifJ_y_R);
  }

protected:

  double tol_ = 1e-8;

  Manifold state;
  Tangent  delta;

  Manifold state_plus_delta_analytic;
  Manifold state_plus_delta_autodiff;
};

} /* namespace manif */

#endif /* _MANIF_MANIF_CERES_TEST_UTILS_H_ */
