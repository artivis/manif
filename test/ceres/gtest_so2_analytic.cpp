#include <gtest/gtest.h>

#include "../test_utils.h"

#include "manif/SO2.h"
#include "manif/ceres/ceres.h"

#include <ceres/ceres.h>

using namespace manif;

TEST(TEST_SO2_CERES, TEST_SO2_OBJECTIVE)
{
  // Create 4 objectives spread arround pi
  ObjectiveSO2 obj_pi_over_4(     M_PI/4.);
  ObjectiveSO2 obj_3_pi_over_8(3.*M_PI/8.);
  ObjectiveSO2 obj_5_pi_over_8(5.*M_PI/8.);
  ObjectiveSO2 obj_3_pi_over_4(3.*M_PI/4.);

  /// @todo eval Jac
////  double** jacobians = new double*[10];
//  for (int i = 0; i < 2; ++i) {
////    jacobians[i] = new double[1];
//  }

  SO2d average_state(0);

  double residuals = 0.0;

  double*  parameter;
  double** parameters;
  parameter  = average_state.data();
  parameters = &parameter;

  obj_pi_over_4.Evaluate(parameters, &residuals, nullptr);
  EXPECT_DOUBLE_EQ(1.*M_PI/4., residuals);

  obj_3_pi_over_8.Evaluate(parameters, &residuals, nullptr);
  EXPECT_DOUBLE_EQ(3.*M_PI/8., residuals);

  obj_5_pi_over_8.Evaluate(parameters, &residuals, nullptr);
  EXPECT_DOUBLE_EQ(5.*M_PI/8., residuals);

  obj_3_pi_over_4.Evaluate(parameters, &residuals, nullptr );
  EXPECT_DOUBLE_EQ(3.*M_PI/4., residuals);
}

TEST(TEST_SO2_CERES, TEST_SO2_LOCAL_PARAMETRIZATION)
{
  LocalParameterizationSO2 local_parameterization;

  // 0 + pi

  double x[2] = {1.0, 0.0};
  double delta[1] = {M_PI};
  double x_plus_delta[2] = {0.0, 0.0};

  local_parameterization.Plus(x, delta, x_plus_delta);

  EXPECT_DOUBLE_EQ(-1.0, x_plus_delta[0]);
  EXPECT_NEAR(0.0, x_plus_delta[1], 1e-15);

  EXPECT_EQ(M_PI, Eigen::Map<const SO2d>(x_plus_delta).angle());

  // pi/4 + pi

  Eigen::Map<SO2d> map_so2(x);
  map_so2 = SO2d(M_PI/4.);

  delta[0] = M_PI;
  x_plus_delta[0] = 0;
  x_plus_delta[1] = 0;

  local_parameterization.Plus(x, delta, x_plus_delta);

  EXPECT_DOUBLE_EQ(cos(-3.*M_PI/4.), x_plus_delta[0]);
  EXPECT_DOUBLE_EQ(sin(-3.*M_PI/4.), x_plus_delta[1]);

  EXPECT_NEAR(-3.*M_PI/4., Eigen::Map<const SO2d>(x_plus_delta).angle(), 1e-15);

//  double J_rplus[2];
//  local_parameterization.ComputeJacobian(x, J_rplus);

//  EXPECT_DOUBLE_EQ(-0.70710678118654746, J_rplus[0]);
//  EXPECT_DOUBLE_EQ( 0.70710678118654757, J_rplus[1]);
}

TEST(TEST_SO2_CERES, TEST_JACOBIANS)
{
  SO2d state(M_PI/4.);
  SO2Tangentd delta(M_PI);

  SO2d state_plus_delta_analytic(0);
  SO2d state_plus_delta_autodiff(0);

  // expected state+delta
  SO2d state_plus_delta_result(-3.*M_PI/4.);

  // Analytic

  ObjectiveSO2 analytic_obj_pi_over_4(M_PI/4.);

  double*  parameter;
  double** parameters;
  parameter  = state.data();
  parameters = &parameter;
  double residuals = 0.0;

  ObjectiveSO2::Jacobian analyticJ_y_r;

  double*  jacobian;
  double** jacobians;
  jacobian  = analyticJ_y_r.data();
  jacobians = &jacobian;

  analytic_obj_pi_over_4.Evaluate(parameters, &residuals, jacobians);
  EXPECT_DOUBLE_EQ(0, residuals);

  LocalParameterizationSO2 analytic_local_parameterization;

  analytic_local_parameterization.Plus(state.data(), delta.data(),
                                       state_plus_delta_analytic.data());

  LocalParameterizationSO2::Jacobian analyticJ_r_R;
  analytic_local_parameterization.ComputeJacobian(state.data(),
                                                  analyticJ_r_R.data());

  // Autodiff

  std::shared_ptr<ceres::CostFunction> autodiff_obj_pi_over_4 =
      make_objective_autodiff<SO2d>(M_PI/4.);

  ObjectiveSO2::Jacobian autodiffJ_y_r;

  jacobian = autodiffJ_y_r.data();

  autodiff_obj_pi_over_4->Evaluate(parameters, &residuals, jacobians);
  EXPECT_DOUBLE_EQ(0, residuals);

  std::shared_ptr<ceres::LocalParameterization>
    auto_diff_local_parameterization =
      make_local_parametrization_autodiff<SO2d>();

  auto_diff_local_parameterization->Plus(state.data(), delta.data(),
                                         state_plus_delta_autodiff.data());

  LocalParameterizationSO2::Jacobian autodiffJ_r_R;
  auto_diff_local_parameterization->ComputeJacobian(state.data(),
                                                    autodiffJ_r_R.data());

  EXPECT_MANIF_NEAR(state_plus_delta_analytic,
                    state_plus_delta_result);
  EXPECT_MANIF_NEAR(state_plus_delta_analytic,
                    state_plus_delta_autodiff);

  SO2d::Jacobian analyticJ_y_R = analyticJ_y_r * analyticJ_r_R;
  SO2d::Jacobian autodiffJ_y_R = autodiffJ_y_r * autodiffJ_r_R;

  EXPECT_EIGEN_NEAR(analyticJ_y_R, autodiffJ_y_R);
}

TEST(TEST_SO2_CERES, TEST_SO2_SMALL_PROBLEM)
{
  // Tell ceres not to take ownership of the raw pointers
  ceres::Problem::Options problem_options;
  problem_options.cost_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  problem_options.local_parameterization_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;

  ceres::Problem problem(problem_options);

  SO2d average_state(M_PI/4.);

  // Create 4 objectives spread arround pi
  ObjectiveSO2 obj_pi_over_4(  SO2d(   M_PI/4.)),
               obj_3_pi_over_8(SO2d(3.*M_PI/8.)),
               obj_5_pi_over_8(SO2d(5.*M_PI/8.)),
               obj_3_pi_over_4(SO2d(3.*M_PI/4.));

  // Add residual blocks to ceres problem
  problem.AddResidualBlock( &obj_pi_over_4,
                            nullptr,
                            average_state.data() );

  problem.AddResidualBlock( &obj_3_pi_over_8,
                            nullptr,
                            average_state.data() );

  problem.AddResidualBlock( &obj_5_pi_over_8,
                            nullptr,
                             average_state.data() );

  problem.AddResidualBlock( &obj_3_pi_over_4,
                            nullptr,
                            average_state.data() );

  LocalParameterizationSO2 local_parametrization;

  problem.SetParameterization( average_state.data(),
                               &local_parametrization );

  std::cout << "-----------------------------\n";
  std::cout << "|       Calling Solve !     |\n";
  std::cout << "-----------------------------\n\n";

  // Run the solver!
  ceres::Solver::Options options;
  options.function_tolerance = 1e-10;
  options.minimizer_progress_to_stdout = true;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  std::cout << "summary:\n" << summary.BriefReport() << "\n";
//  std::cout << "summary:\n" << summary.FullReport() << "\n";

  ASSERT_TRUE(summary.IsSolutionUsable());

  EXPECT_ANGLE_NEAR(M_PI_2, average_state.angle(), 1e-8);
}

TEST(TEST_SO2_CERES, TEST_SO2_CONSTRAINT)
{
  // Tell ceres not to take ownership of the raw pointers
  ceres::Problem::Options problem_options;
  problem_options.cost_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  problem_options.local_parameterization_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;

  ceres::Problem problem(problem_options);

  GaussianNoiseGenerator<> noise(0, 0.1);

  //  p0 expected at  0
  //  p1 expected at  M_PI/4.
  //  p2 expected at  M_PI_2
  //  p3 expected at  3.*M_PI/4.
  //  p4 expected at  M_PI
  //  p5 expected at -3.*M_PI/4
  //  p6 expected at -M_PI_2
  //  p7 expected at -M_PI/4.

  SO2d state_0( /*0*/          + noise());
  SO2d state_1( /*M_PI/4.*/    + noise());
  SO2d state_2( /*M_PI_2*/     + noise());
  SO2d state_3( /*3.*M_PI/4.*/ + noise());
  SO2d state_4( /*M_PI*/       + noise());
  SO2d state_5(/*-3.*M_PI/4 */ + noise());
  SO2d state_6(/*-M_PI_2*/     + noise());
  SO2d state_7(/*-M_PI/4.*/    + noise());

  std::cout << "Initial states :\n";
  std::cout << "p0 : [" << state_0.angle() << "]\n";
  std::cout << "p1 : [" << state_1.angle() << "]\n";
  std::cout << "p2 : [" << state_2.angle() << "]\n";
  std::cout << "p3 : [" << state_3.angle() << "]\n";
  std::cout << "p4 : [" << state_4.angle() << "]\n";
  std::cout << "p5 : [" << state_5.angle() << "]\n";
  std::cout << "p6 : [" << state_6.angle() << "]\n";
  std::cout << "p7 : [" << state_7.angle() << "]\n";
  std::cout << "\n";

  ConstraintSO2 constraint_0_1(M_PI/4.);
  ConstraintSO2 constraint_1_2(M_PI/4.);
  ConstraintSO2 constraint_2_3(M_PI/4.);
  ConstraintSO2 constraint_3_4(M_PI/4.);
  ConstraintSO2 constraint_4_5(M_PI/4.);
  ConstraintSO2 constraint_5_6(M_PI/4.);
  ConstraintSO2 constraint_6_7(M_PI/4.);
//  ConstraintSO2 constraint_7_0(M_PI/4.);

  // Add residual blocks to ceres problem
  problem.AddResidualBlock( &constraint_0_1,
                            nullptr,
                            state_0.data(), state_1.data() );

  problem.AddResidualBlock( &constraint_1_2,
                            nullptr,
                            state_1.data(), state_2.data() );

  problem.AddResidualBlock( &constraint_2_3,
                            nullptr,
                            state_2.data(), state_3.data() );

  problem.AddResidualBlock( &constraint_3_4,
                            nullptr,
                            state_3.data(), state_4.data() );

  problem.AddResidualBlock( &constraint_4_5,
                            nullptr,
                            state_4.data(), state_5.data() );

  problem.AddResidualBlock( &constraint_5_6,
                            nullptr,
                            state_5.data(), state_6.data() );

  problem.AddResidualBlock( &constraint_6_7,
                            nullptr,
                            state_6.data(), state_7.data() );

//  problem.AddResidualBlock( &constraint_7_0,
//                            nullptr,
//                            state_7.data(), state_0.data() );

  // Anchor state 0 at 0
  ObjectiveSO2 obj_origin(0);

  problem.AddResidualBlock( &obj_origin,
                            nullptr,
                            state_0.data() );

  /// @todo Not sure if LocalParametrization is
  /// thread safe thus the several instance
  LocalParameterizationSO2 local_parametrization;

  problem.SetParameterization( state_0.data(),
                               &local_parametrization );

  problem.SetParameterization( state_1.data(),
                               &local_parametrization );

  problem.SetParameterization( state_2.data(),
                               &local_parametrization );

  problem.SetParameterization( state_3.data(),
                               &local_parametrization );

  problem.SetParameterization( state_4.data(),
                               &local_parametrization );

  problem.SetParameterization( state_5.data(),
                               &local_parametrization );

  problem.SetParameterization( state_6.data(),
                               &local_parametrization );

  problem.SetParameterization( state_7.data(),
                               &local_parametrization );

  std::cout << "-----------------------------\n";
  std::cout << "|       Calling Solve !     |\n";
  std::cout << "-----------------------------\n\n";

  // Run the solver!
  ceres::Solver::Options options;
  options.function_tolerance = 1e-15;
  options.minimizer_progress_to_stdout = true;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  std::cout << "summary:\n" << summary.BriefReport() << "\n";
//  std::cout << "summary:\n" << summary.FullReport() << "\n";

  ASSERT_TRUE(summary.IsSolutionUsable());

  std::cout << "Final states :\n";
  std::cout << "p0 : [" << state_0.angle() << "]\n";
  std::cout << "p1 : [" << state_1.angle() << "]\n";
  std::cout << "p2 : [" << state_2.angle() << "]\n";
  std::cout << "p3 : [" << state_3.angle() << "]\n";
  std::cout << "p4 : [" << state_4.angle() << "]\n";
  std::cout << "p5 : [" << state_5.angle() << "]\n";
  std::cout << "p6 : [" << state_6.angle() << "]\n";
  std::cout << "p7 : [" << state_7.angle() << "]\n";
  std::cout << "\n";

  constexpr double ceres_eps = 1e-6;

  EXPECT_ANGLE_NEAR(0,          state_0.angle(), ceres_eps);
  EXPECT_ANGLE_NEAR(M_PI/4.,    state_1.angle(), ceres_eps);
  EXPECT_ANGLE_NEAR(M_PI_2,     state_2.angle(), ceres_eps);
  EXPECT_ANGLE_NEAR(3.*M_PI/4., state_3.angle(), ceres_eps);
  EXPECT_ANGLE_NEAR(M_PI,       state_4.angle(), ceres_eps);
  EXPECT_ANGLE_NEAR(-3.*M_PI/4, state_5.angle(), ceres_eps);
  EXPECT_ANGLE_NEAR(-M_PI_2,    state_6.angle(), ceres_eps);
  EXPECT_ANGLE_NEAR(-M_PI/4.,   state_7.angle(), ceres_eps);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
