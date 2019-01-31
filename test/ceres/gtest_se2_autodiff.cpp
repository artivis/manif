#include <gtest/gtest.h>

#include "manif/SE2.h"

#include "manif/impl/utils.h"
#include "../test_utils.h"

#include "manif/ceres/ceres.h"

#include <ceres/ceres.h>

namespace manif {

using LocalParameterizationSE2 = CeresLocalParameterizationSE2;
using ObjectiveSE2  = CeresObjectiveSE2;
using ConstraintSE2 = CeresConstraintSE2;

} /* namespace manif */

using namespace manif;

TEST(TEST_LOCAL_PARAMETRIZATION, TEST_SE2_OBJECTIVE_AUTODIFF)
{
  // Create 4 objectives spread arround pi
  std::shared_ptr<ceres::CostFunction> obj_pi_over_4 =
      make_objective_autodiff<SE2d>(3,3,M_PI/4.);

  std::shared_ptr<ceres::CostFunction> obj_3_pi_over_8 =
      make_objective_autodiff<SE2d>(3,1,3.*M_PI/8.);

  std::shared_ptr<ceres::CostFunction> obj_5_pi_over_8 =
      make_objective_autodiff<SE2d>(1,1,5.*M_PI/8.);

  std::shared_ptr<ceres::CostFunction> obj_3_pi_over_4 =
      make_objective_autodiff<SE2d>(1,3,3.*M_PI/4.);

  /// @todo eval Jac
////  double** jacobians = new double*[10];
//  for (int i = 0; i < 2; ++i) {
////    jacobians[i] = new double[1];
//  }

  SE2d average_state(0,0,0);

  double residuals[3] = {0,0,0};

  double*  parameter;
  double** parameters;
  parameter  = average_state.data();
  parameters = &parameter;

  obj_pi_over_4->Evaluate(parameters, residuals, nullptr);

  /// @todo
//  EXPECT_DOUBLE_EQ(d0, residuals[0]);
//  EXPECT_DOUBLE_EQ(d0, residuals[1]);
//  EXPECT_DOUBLE_EQ(1.*M_PI/4., residuals[2]);

  obj_3_pi_over_8->Evaluate(parameters, residuals, nullptr);
//  EXPECT_DOUBLE_EQ(3, residuals[0]);
//  EXPECT_DOUBLE_EQ(1, residuals[1]);
//  EXPECT_DOUBLE_EQ(3.*M_PI/8., residuals[2]);

  obj_5_pi_over_8->Evaluate(parameters, residuals, nullptr);
//  EXPECT_DOUBLE_EQ(1, residuals[0]);
//  EXPECT_DOUBLE_EQ(3, residuals[1]);
//  EXPECT_DOUBLE_EQ(5.*M_PI/8., residuals[2]);

  obj_3_pi_over_4->Evaluate(parameters, residuals, nullptr );
//  EXPECT_DOUBLE_EQ(1, residuals[0]);
//  EXPECT_DOUBLE_EQ(1, residuals[1]);
//  EXPECT_DOUBLE_EQ(3.*M_PI/4., residuals[2]);
}

TEST(TEST_LOCAL_PARAMETRIZATION, TEST_SE2_LOCAL_PARAMETRIZATION_AUTODIFF)
{
  std::shared_ptr<ceres::LocalParameterization>
    auto_diff_local_parameterization =
      make_local_parameterization_autodiff<SE2d>();

  // 0 + pi

  double x[SE2d::RepSize] = {0.0, 0.0, 1.0, 0.0};
  double delta[SE2Tangentd::RepSize] = {1.0, 1.0, M_PI};
  double x_plus_delta[SE2d::RepSize] = {0.0, 0.0, 0.0, 0.0};

  auto_diff_local_parameterization->Plus(x, delta, x_plus_delta);

  /// @todo
//  EXPECT_DOUBLE_EQ(-1.0, x_plus_delta[0]);
//  EXPECT_DOUBLE_EQ(-1.0, x_plus_delta[1]);

  EXPECT_DOUBLE_EQ(-1.0, x_plus_delta[2]);
  EXPECT_NEAR(0.0, x_plus_delta[3], 1e-15);

  EXPECT_EQ(M_PI, Eigen::Map<const SE2d>(x_plus_delta).angle());

  // pi/4 + pi

  Eigen::Map<SE2d> map_se2(x);
  map_se2 = SE2d(1, 1, M_PI/4.);

  delta[0] = 1;
  delta[1] = 2;
  delta[2] = M_PI;
  x_plus_delta[0] = 0;
  x_plus_delta[1] = 0;
  x_plus_delta[2] = 1;
  x_plus_delta[3] = 0;

  auto_diff_local_parameterization->Plus(x, delta, x_plus_delta);

  /// @todo
//  EXPECT_DOUBLE_EQ(-1.0, x_plus_delta[0]);
//  EXPECT_DOUBLE_EQ(-1.0, x_plus_delta[1]);

  EXPECT_DOUBLE_EQ(cos(-3.*M_PI/4.), x_plus_delta[2]);
  EXPECT_DOUBLE_EQ(sin(-3.*M_PI/4.), x_plus_delta[3]);

  EXPECT_NEAR(-3.*M_PI/4., Eigen::Map<const SE2d>(x_plus_delta).angle(), 1e-15);

  double J_rplus[SE2d::RepSize*SE2Tangentd::RepSize];
  auto_diff_local_parameterization->ComputeJacobian(x, J_rplus);

  /// @todo values copied from terminal...
  EXPECT_DOUBLE_EQ( 0.70710678118654746, J_rplus[0]);
  EXPECT_DOUBLE_EQ(-0.70710678118654757, J_rplus[1]);
  EXPECT_DOUBLE_EQ( 0, J_rplus[2]);
  EXPECT_DOUBLE_EQ( 0.70710678118654746, J_rplus[3]);
  EXPECT_DOUBLE_EQ( 0.70710678118654746, J_rplus[4]);
  EXPECT_DOUBLE_EQ( 0, J_rplus[5]);
  EXPECT_DOUBLE_EQ( 0, J_rplus[6]);
  EXPECT_DOUBLE_EQ( 0, J_rplus[7]);
  EXPECT_DOUBLE_EQ(-0.70710678118654746, J_rplus[8]);
  EXPECT_DOUBLE_EQ( 0, J_rplus[9]);
  EXPECT_DOUBLE_EQ( 0, J_rplus[10]);
  EXPECT_DOUBLE_EQ( 0.70710678118654757, J_rplus[11]);
}

TEST(TEST_LOCAL_PARAMETRIZATION, TEST_SE2_SMALL_PROBLEM_AUTODIFF)
{
  // Tell ceres not to take ownership of the raw pointers
  ceres::Problem::Options problem_options;
  problem_options.cost_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  problem_options.local_parameterization_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;

  ceres::Problem problem(problem_options);

  // Create 4 objectives spread arround pi
  std::shared_ptr<ceres::CostFunction> obj_pi_over_4 =
      make_objective_autodiff<SE2d>(3,3,M_PI/4.);

  std::shared_ptr<ceres::CostFunction> obj_3_pi_over_8 =
      make_objective_autodiff<SE2d>(3,1,3.*M_PI/8.);

  std::shared_ptr<ceres::CostFunction> obj_5_pi_over_8 =
      make_objective_autodiff<SE2d>(1,1,5.*M_PI/8.);

  std::shared_ptr<ceres::CostFunction> obj_3_pi_over_4 =
      make_objective_autodiff<SE2d>(1,3,3.*M_PI/4.);

  SE2d average_state(0,0,0);

  /////////////////////////////////

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
      make_local_parameterization_autodiff<SE2d>();

  problem.SetParameterization( average_state.data(),
                               auto_diff_local_parameterization.get() );

  std::cout << "-----------------------------\n";
  std::cout << "|       Calling Solve !     |\n";
  std::cout << "-----------------------------\n\n";

  // Initializing state closer to solution
//  average_state = SE2d(3.*M_PI/8.);

  // Run the solver!
  ceres::Solver::Options options;
  options.function_tolerance = 1e-15;
  options.minimizer_progress_to_stdout = true;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  std::cout << "summary:\n" << summary.BriefReport() << "\n\n";
//  std::cout << "summary:\n" << summary.FullReport() << "\n";

  std::cout << "Final state:\nx:" << average_state.x()
            << "\ny:" << average_state.y()
            << "\nt:" << average_state.angle()
            << "\n\n";

  EXPECT_TRUE(summary.IsSolutionUsable());

  EXPECT_NEAR(2,      average_state.x(),     1e-1);
  EXPECT_NEAR(2,      average_state.y(),     1e-1);
  EXPECT_NEAR(M_PI_2, average_state.angle(), 1e-1);
}


TEST(TEST_LOCAL_PARAMETRIZATION, TEST_SE2_CONSTRAINT_AUTODIFF)
{
  // Tell ceres not to take ownership of the raw pointers
  ceres::Problem::Options problem_options;
  problem_options.cost_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  problem_options.local_parameterization_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;

  //
  //    5 ____ 4
  //   6 /    \ 3
  //     |    |
  //   7 \    / 2
  //      ----
  //     0    1

  ceres::Problem problem(problem_options);

//  p0 expected at  0, 0, -M_PI/4.
//  p1 expected at  1, 0, 0
//  p2 expected at  2, 1, M_PI/4.
//  p3 expected at  2, 2, M_PI/2.
//  p4 expected at  1, 3, 3.*M_PI/4.
//  p5 expected at  0, 3, M_PI
//  p6 expected at -1, 2, -3.*M_PI/4.
//  p7 expected at -1, 1, -M_PI/2.

  GaussianNoiseGenerator<> noise(0, 0.1);

  SE2d state_0( 0 + noise(), 0 + noise(), -M_PI/4.    + noise());
  SE2d state_1( 1 + noise(), 0 + noise(), 0           + noise());
  SE2d state_2( 2 + noise(), 1 + noise(), M_PI/4.     + noise());
  SE2d state_3( 2 + noise(), 2 + noise(), M_PI/2.     + noise());
  SE2d state_4( 1 + noise(), 3 + noise(), 3.*M_PI/4.  + noise());
  SE2d state_5( 0 + noise(), 3 + noise(), M_PI        + noise());
  SE2d state_6(-1 + noise(), 2 + noise(), -3.*M_PI/4. + noise());
  SE2d state_7(-1 + noise(), 1 + noise(), -M_PI/2.    + noise());

  std::cout << "Initial states :\n";
  std::cout << "p0 : [" << state_0.x() << "," << state_0.y() << "," << state_0.angle() << "]\n";
  std::cout << "p1 : [" << state_1.x() << "," << state_1.y() << "," << state_1.angle() << "]\n";
  std::cout << "p2 : [" << state_2.x() << "," << state_2.y() << "," << state_2.angle() << "]\n";
  std::cout << "p3 : [" << state_3.x() << "," << state_3.y() << "," << state_3.angle() << "]\n";
  std::cout << "p4 : [" << state_4.x() << "," << state_4.y() << "," << state_4.angle() << "]\n";
  std::cout << "p5 : [" << state_5.x() << "," << state_5.y() << "," << state_5.angle() << "]\n";
  std::cout << "p6 : [" << state_6.x() << "," << state_6.y() << "," << state_6.angle() << "]\n";
  std::cout << "p7 : [" << state_7.x() << "," << state_7.y() << "," << state_7.angle() << "]\n";
  std::cout << "\n";

  double inv_sqrt_2 = 1./sqrt(2.);

  auto constraint_0_1 = make_constraint_autodiff<SE2d>( SE2d( inv_sqrt_2, inv_sqrt_2, M_PI/4. ).log() );
  auto constraint_1_2 = make_constraint_autodiff<SE2d>( SE2d( 1,          1,          M_PI/4. ).log() );
  auto constraint_2_3 = make_constraint_autodiff<SE2d>( SE2d( inv_sqrt_2, inv_sqrt_2, M_PI/4. ).log() );
  auto constraint_3_4 = make_constraint_autodiff<SE2d>( SE2d( 1,          1,          M_PI/4. ).log() );
  auto constraint_4_5 = make_constraint_autodiff<SE2d>( SE2d( inv_sqrt_2, inv_sqrt_2, M_PI/4. ).log() );
  auto constraint_5_6 = make_constraint_autodiff<SE2d>( SE2d( 1,          1,          M_PI/4. ).log() );
  auto constraint_6_7 = make_constraint_autodiff<SE2d>( SE2d( inv_sqrt_2, inv_sqrt_2, M_PI/4. ).log() );
  auto constraint_7_0 = make_constraint_autodiff<SE2d>( SE2d( 1,          1,          M_PI/4. ).log() );

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

  // Anchor on state
  std::shared_ptr<ceres::CostFunction> obj_origin =
      make_objective_autodiff<SE2d>(0,0,-M_PI/4.);

  problem.AddResidualBlock( obj_origin.get(),
                            nullptr,
                            state_0.data() );

  std::shared_ptr<ceres::LocalParameterization>
    auto_diff_local_parameterization =
      make_local_parameterization_autodiff<SE2d>();

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

  std::cout << "summary:\n" << summary.BriefReport() << "\n\n";
//  std::cout << "summary:\n" << summary.FullReport() << "\n";

  ASSERT_TRUE(summary.IsSolutionUsable());

  std::cout << "Final states :\n";
  std::cout << "p0 : [" << state_0.x() << "," << state_0.y() << "," << state_0.angle() << "]\n";
  std::cout << "p1 : [" << state_1.x() << "," << state_1.y() << "," << state_1.angle() << "]\n";
  std::cout << "p2 : [" << state_2.x() << "," << state_2.y() << "," << state_2.angle() << "]\n";
  std::cout << "p3 : [" << state_3.x() << "," << state_3.y() << "," << state_3.angle() << "]\n";
  std::cout << "p4 : [" << state_4.x() << "," << state_4.y() << "," << state_4.angle() << "]\n";
  std::cout << "p5 : [" << state_5.x() << "," << state_5.y() << "," << state_5.angle() << "]\n";
  std::cout << "p6 : [" << state_6.x() << "," << state_6.y() << "," << state_6.angle() << "]\n";
  std::cout << "p7 : [" << state_7.x() << "," << state_7.y() << "," << state_7.angle() << "]\n";

  constexpr double ceres_eps = 2e-3;

  EXPECT_NEAR( 0,                 state_0.x(),      ceres_eps);
  EXPECT_NEAR( 0,                 state_0.y(),      ceres_eps);
  EXPECT_ANGLE_NEAR(-M_PI/4.,     state_0.angle(),  ceres_eps);

  EXPECT_NEAR( 1,                 state_1.x(),      ceres_eps);
  EXPECT_NEAR( 0,                 state_1.y(),      ceres_eps);
  EXPECT_ANGLE_NEAR( 0,           state_1.angle(),  ceres_eps);

  EXPECT_NEAR( 2,                 state_2.x(),      ceres_eps);
  EXPECT_NEAR( 1,                 state_2.y(),      ceres_eps);
  EXPECT_ANGLE_NEAR( M_PI/4.,     state_2.angle(),  ceres_eps);

  EXPECT_NEAR( 2,                 state_3.x(),      ceres_eps);
  EXPECT_NEAR( 2,                 state_3.y(),      ceres_eps);
  EXPECT_ANGLE_NEAR( M_PI_2,      state_3.angle(),  ceres_eps);

  EXPECT_NEAR( 1,                 state_4.x(),      ceres_eps);
  EXPECT_NEAR( 3,                 state_4.y(),      ceres_eps);
  EXPECT_ANGLE_NEAR( 3.*M_PI/4.,  state_4.angle(),  ceres_eps);

  EXPECT_NEAR( 0,                 state_5.x(),      ceres_eps);
  EXPECT_NEAR( 3,                 state_5.y(),      ceres_eps);
  EXPECT_ANGLE_NEAR(-M_PI,        state_5.angle(),  ceres_eps);

  EXPECT_NEAR(-1,                 state_6.x(),      ceres_eps);
  EXPECT_NEAR( 2,                 state_6.y(),      ceres_eps);
  EXPECT_ANGLE_NEAR(-3.*M_PI/4,   state_6.angle(),  ceres_eps);

  EXPECT_NEAR(-1,                 state_7.x(),      ceres_eps);
  EXPECT_NEAR( 1,                 state_7.y(),      ceres_eps);
  EXPECT_ANGLE_NEAR(-M_PI_2,      state_7.angle(),  ceres_eps);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
