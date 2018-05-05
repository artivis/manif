#include <gtest/gtest.h>

#include "../test_utils.h"

#include "manif/SO2.h"
#include "manif/ceres/local_parametrization.h"
#include "manif/ceres/objective.h"
#include "manif/ceres/constraint.h"

#include "manif/ceres/ceres_utils.h"

#include <ceres/ceres.h>

namespace manif
{

using LocalParameterizationSO2 = LocalParameterization<SO2d>;
using ObjectiveSO2  = Objective<SO2d>;
using ConstraintSO2 = Constraint<SO2d>;

} /* namespace manif */

using namespace manif;

TEST(TEST_LOCAL_PARAMETRIZATION, TEST_SO2_AUTODIFF_OBJECTIVE)
{
  // Create 4 objectives spread arround pi
  std::shared_ptr<ceres::CostFunction> obj_pi_over_4 =
      make_objective_autodiff<SO2d>(M_PI/4.);

  std::shared_ptr<ceres::CostFunction> obj_3_pi_over_8 =
      make_objective_autodiff<SO2d>(3.*M_PI/8.);

  std::shared_ptr<ceres::CostFunction> obj_5_pi_over_8 =
      make_objective_autodiff<SO2d>(5.*M_PI/8.);

  std::shared_ptr<ceres::CostFunction> obj_3_pi_over_4 =
      make_objective_autodiff<SO2d>(3.*M_PI/4.);

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

  obj_pi_over_4->Evaluate(parameters, &residuals, nullptr);
  EXPECT_DOUBLE_EQ(1.*M_PI/4., residuals);

  obj_3_pi_over_8->Evaluate(parameters, &residuals, nullptr);
  EXPECT_DOUBLE_EQ(3.*M_PI/8., residuals);

  obj_5_pi_over_8->Evaluate(parameters, &residuals, nullptr);
  EXPECT_DOUBLE_EQ(5.*M_PI/8., residuals);

  obj_3_pi_over_4->Evaluate(parameters, &residuals, nullptr );
  EXPECT_DOUBLE_EQ(3.*M_PI/4., residuals);
}

TEST(TEST_LOCAL_PARAMETRIZATION, TEST_SO2_AUTODIFF_LOCAL_PARAMETRIZATION)
{
  std::shared_ptr<ceres::LocalParameterization>
    auto_diff_local_parameterization =
      make_local_parametrization_autodiff<SO2d>();

  // 0 + pi

  double x[2] = {1.0, 0.0};
  double delta[1] = {M_PI};
  double x_plus_delta[2] = {0.0, 0.0};

  auto_diff_local_parameterization->Plus(x, delta, x_plus_delta);

  EXPECT_DOUBLE_EQ(-1.0, x_plus_delta[0]);
  EXPECT_NEAR(0.0, x_plus_delta[1], 1e-15);

  EXPECT_EQ(M_PI, Eigen::Map<const SO2d>(x_plus_delta).angle());

  // pi/4 + pi

  Eigen::Map<SO2d> map_so2(x);
  map_so2 = SO2d(M_PI/4.);

  delta[0] = M_PI;
  x_plus_delta[0] = 0;
  x_plus_delta[1] = 0;

  auto_diff_local_parameterization->Plus(x, delta, x_plus_delta);

  EXPECT_DOUBLE_EQ(cos(-3.*M_PI/4.), x_plus_delta[0]);
  EXPECT_DOUBLE_EQ(sin(-3.*M_PI/4.), x_plus_delta[1]);

  EXPECT_NEAR(-3.*M_PI/4., Eigen::Map<const SO2d>(x_plus_delta).angle(), 1e-15);

  double J_rplus[2];
  auto_diff_local_parameterization->ComputeJacobian(x, J_rplus);

  EXPECT_DOUBLE_EQ(-0.70710678118654746, J_rplus[0]);
  EXPECT_DOUBLE_EQ( 0.70710678118654757, J_rplus[1]);
}

TEST(TEST_LOCAL_PARAMETRIZATION, TEST_SO2_AUTODIFF_SMALL_PROBLEM)
{
  // Tell ceres not to take ownership of the raw pointers
  ceres::Problem::Options problem_options;
  problem_options.cost_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  problem_options.local_parameterization_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;

  ceres::Problem problem(problem_options);

  // Create 4 objectives spread arround pi
  std::shared_ptr<ceres::CostFunction> obj_pi_over_4 =
      make_objective_autodiff<SO2d>(M_PI/4.);

  std::shared_ptr<ceres::CostFunction> obj_3_pi_over_8 =
      make_objective_autodiff<SO2d>(3.*M_PI/8.);

  std::shared_ptr<ceres::CostFunction> obj_5_pi_over_8 =
      make_objective_autodiff<SO2d>(5.*M_PI/8.);

  std::shared_ptr<ceres::CostFunction> obj_3_pi_over_4 =
      make_objective_autodiff<SO2d>(3.*M_PI/4.);

  // Initializing state close to solution
  SO2d average_state(3.*M_PI/8.);

  // Add residual blocks to ceres problem
  problem.AddResidualBlock( obj_pi_over_4.get(),
                            nullptr,
                            average_state.data() );

  problem.AddResidualBlock( obj_3_pi_over_8.get(),
                            nullptr,
                            average_state.data() );

  problem.AddResidualBlock( obj_5_pi_over_8.get(),
                            nullptr,
                             average_state.data() );

  problem.AddResidualBlock( obj_3_pi_over_4.get(),
                            nullptr,
                            average_state.data() );

  std::shared_ptr<ceres::LocalParameterization>
    auto_diff_local_parameterization =
      make_local_parametrization_autodiff<SO2d>();

  problem.SetParameterization( average_state.data(),
                               auto_diff_local_parameterization.get() );

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

  // 1.3088223838053636e-09
  EXPECT_ANGLE_NEAR(M_PI_2, average_state.angle(), 1e-8);
}


TEST(TEST_LOCAL_PARAMETRIZATION, TEST_SO2_CONSTRAINT_AUTODIFF)
{
  // Tell ceres not to take ownership of the raw pointers
  ceres::Problem::Options problem_options;
  problem_options.cost_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  problem_options.local_parameterization_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;

  // states are equally spread over the circle by
  // M_PI / 4

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

  SO2d state_0( 0          + noise());
  SO2d state_1( M_PI/4.    + noise());
  SO2d state_2( M_PI_2     + noise());
  SO2d state_3( 3.*M_PI/4. + noise());
  SO2d state_4( M_PI       + noise());
  SO2d state_5(-3.*M_PI/4  + noise());
  SO2d state_6(-M_PI_2     + noise());
  SO2d state_7(-M_PI/4.    + noise());

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

  auto constraint_0_1 = make_constraint_autodiff<SO2d>(M_PI/4.);
  auto constraint_1_2 = make_constraint_autodiff<SO2d>(M_PI/4.);
  auto constraint_2_3 = make_constraint_autodiff<SO2d>(M_PI/4.);
  auto constraint_3_4 = make_constraint_autodiff<SO2d>(M_PI/4.);
  auto constraint_4_5 = make_constraint_autodiff<SO2d>(M_PI/4.);
  auto constraint_5_6 = make_constraint_autodiff<SO2d>(M_PI/4.);
  auto constraint_6_7 = make_constraint_autodiff<SO2d>(M_PI/4.);
  auto constraint_7_0 = make_constraint_autodiff<SO2d>(M_PI/4.);

  // Add residual blocks to ceres problem
  problem.AddResidualBlock( constraint_0_1.get(),
                            nullptr,
                            state_0.data(), state_1.data() );

  problem.AddResidualBlock( constraint_1_2.get(),
                            nullptr,
                            state_1.data(), state_2.data() );

  problem.AddResidualBlock( constraint_2_3.get(),
                            nullptr,
                            state_2.data(), state_3.data() );

  problem.AddResidualBlock( constraint_3_4.get(),
                            nullptr,
                            state_3.data(), state_4.data() );

  problem.AddResidualBlock( constraint_4_5.get(),
                            nullptr,
                            state_4.data(), state_5.data() );

  problem.AddResidualBlock( constraint_5_6.get(),
                            nullptr,
                            state_5.data(), state_6.data() );

  problem.AddResidualBlock( constraint_6_7.get(),
                            nullptr,
                            state_6.data(), state_7.data() );

  problem.AddResidualBlock( constraint_7_0.get(),
                            nullptr,
                            state_7.data(), state_0.data() );

  // Anchor state 0 at 0
  std::shared_ptr<ceres::CostFunction> obj_origin =
      make_objective_autodiff<SO2d>(0);

  problem.AddResidualBlock( obj_origin.get(),
                            nullptr,
                            state_0.data() );

  std::shared_ptr<ceres::LocalParameterization>
    auto_diff_local_parameterization =
      make_local_parametrization_autodiff<SO2d>();

  problem.SetParameterization( state_0.data(),
                               auto_diff_local_parameterization.get() );

  problem.SetParameterization( state_1.data(),
                               auto_diff_local_parameterization.get() );

  problem.SetParameterization( state_2.data(),
                               auto_diff_local_parameterization.get() );

  problem.SetParameterization( state_3.data(),
                               auto_diff_local_parameterization.get() );

  problem.SetParameterization( state_4.data(),
                               auto_diff_local_parameterization.get() );

  problem.SetParameterization( state_5.data(),
                               auto_diff_local_parameterization.get() );

  problem.SetParameterization( state_6.data(),
                               auto_diff_local_parameterization.get() );

  problem.SetParameterization( state_7.data(),
                               auto_diff_local_parameterization.get() );

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


/*
TEST(TEST_LOCAL_PARAMETRIZATION, TEST_SO2_OBJECTIVE)
{
  // Tell ceres not to take ownership of the raw pointers
  ceres::Problem::Options problem_options;
  problem_options.cost_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  problem_options.local_parameterization_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;

  ceres::Problem problem(problem_options);

  SO2d average_state(0);

  // Create 4 objectives spread arround pi
  ObjectiveSO2 obj_pi_over_4(SO2d(M_PI/4.)),
               obj_3_pi_over_8(SO2d(3.*M_PI/8.)),
               obj_5_pi_over_8(SO2d(5.*M_PI/8.)),
               obj_3_pi_over_4(SO2d(3.*M_PI/4.));

  ceres::AutoDiffCostFunction<ObjectiveSO2, 1, 2>(
          new MyScalarCostFunctor(1.0));

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

  ceres::AutoDiffLocalParameterization<LocalParameterizationSO2, 2, 1>
        auto_diff_local_parameterization;

//  LocalParameterizationSO2 local_parametrization;

  problem.SetParameterization( average_state.data(),
                               &auto_diff_local_parameterization );

  // Run the solver!
  ceres::Solver::Options options;
//  options.max_num_iterations = 50;
//  options.minimizer_progress_to_stdout = false;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  std::cout << "summary:\n" << summary.BriefReport() << "\n";
//  std::cout << "summary:\n" << summary.FullReport() << "\n";

  bool opt_success = (summary.termination_type != 0) and // DID_NOT_RUN
                     (summary.termination_type != 1) and // NO_CONVERGENCE
                     (summary.termination_type != 5);    // NUMERICAL_FAILURE

  EXPECT_TRUE(opt_success) << "Solving failure : "
                           << getReason(summary.termination_type);

  EXPECT_DOUBLE_EQ(M_PI_2, average_state.angle());
}
*/

/*
TEST(TEST_LOCAL_PARAMETRIZATION, TEST_SO2_CONSTRAINT)
{
  // Tell ceres not to take ownership of the raw pointers
  ceres::Problem::Options problem_options;
  problem_options.cost_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;

  ceres::Problem problem(problem_options);

  SO2d state_0(0);
  SO2d state_1(0);
  SO2d state_2(0);
  SO2d state_3(0);
  SO2d state_4(0);
  SO2d state_5(0);
  SO2d state_6(0);
  SO2d state_7(0);

  ConstraintSO2 constraint_0_1(M_PI/4.);
  ConstraintSO2 constraint_1_2(M_PI/4.);
  ConstraintSO2 constraint_2_3(M_PI/4.);
  ConstraintSO2 constraint_3_4(M_PI/4.);
  ConstraintSO2 constraint_4_5(M_PI/4.);
  ConstraintSO2 constraint_5_6(M_PI/4.);
  ConstraintSO2 constraint_6_7(M_PI/4.);

  // This would be like a loop-closure
//  ConstraintSO2 constraint_7_8(M_PI/4.);

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

  LocalParameterizationSO2 local_parametrization_0;
  LocalParameterizationSO2 local_parametrization_1;
  LocalParameterizationSO2 local_parametrization_2;
  LocalParameterizationSO2 local_parametrization_3;
  LocalParameterizationSO2 local_parametrization_4;
  LocalParameterizationSO2 local_parametrization_5;
  LocalParameterizationSO2 local_parametrization_6;
  LocalParameterizationSO2 local_parametrization_7;

  problem.SetParameterization( state_0.data(),
                               &local_parametrization_0 );

  problem.SetParameterization( state_1.data(),
                               &local_parametrization_1 );

  problem.SetParameterization( state_2.data(),
                               &local_parametrization_2 );

  problem.SetParameterization( state_3.data(),
                               &local_parametrization_3 );

  problem.SetParameterization( state_4.data(),
                               &local_parametrization_4 );

  problem.SetParameterization( state_5.data(),
                               &local_parametrization_5 );

  problem.SetParameterization( state_6.data(),
                               &local_parametrization_6 );

  problem.SetParameterization( state_7.data(),
                               &local_parametrization_7 );

  // Run the solver!
  ceres::Solver::Options options;
//  options.max_num_iterations = 50;
//  options.minimizer_progress_to_stdout = false;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

//  std::cout << "summary:\n" << summary.BriefReport() << "\n";
  std::cout << "summary:\n" << summary.FullReport() << "\n";

  bool opt_success = (summary.termination_type != 0) and // DID_NOT_RUN
                     (summary.termination_type != 1) and // NO_CONVERGENCE
                     (summary.termination_type != 5);    // NUMERICAL_FAILURE

  EXPECT_TRUE(opt_success) << getReason(summary.termination_type);
}
*/

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
