#include "manif/SO2.h"
#include "ceres_test_utils.h"

#include <ceres/ceres.h>

using namespace manif;

TEST(TEST_SO2_CERES, TEST_SO2_OBJECTIVE_AUTODIFF)
{
  // Create 4 objectives spread arround pi
  std::shared_ptr<ceres::CostFunction> obj_pi_over_4 =
      make_objective_autodiff<SO2d>(MANIF_PI/4.);

  std::shared_ptr<ceres::CostFunction> obj_3_pi_over_8 =
      make_objective_autodiff<SO2d>(3.*MANIF_PI/8.);

  std::shared_ptr<ceres::CostFunction> obj_5_pi_over_8 =
      make_objective_autodiff<SO2d>(5.*MANIF_PI/8.);

  std::shared_ptr<ceres::CostFunction> obj_3_pi_over_4 =
      make_objective_autodiff<SO2d>(3.*MANIF_PI/4.);

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
  EXPECT_DOUBLE_EQ(1.*MANIF_PI/4., residuals);

  obj_3_pi_over_8->Evaluate(parameters, &residuals, nullptr);
  EXPECT_DOUBLE_EQ(3.*MANIF_PI/8., residuals);

  obj_5_pi_over_8->Evaluate(parameters, &residuals, nullptr);
  EXPECT_DOUBLE_EQ(5.*MANIF_PI/8., residuals);

  obj_3_pi_over_4->Evaluate(parameters, &residuals, nullptr );
  EXPECT_DOUBLE_EQ(3.*MANIF_PI/4., residuals);
}

TEST(TEST_SO2_CERES, TEST_SO2_LOCAL_PARAMETRIZATION_AUTODIFF)
{
#if CERES_VERSION_MAJOR >= 2 && CERES_VERSION_MINOR >= 2
  std::shared_ptr<ceres::Manifold>
    auto_diff_manifold = make_manifold_autodiff<SO2d>();
#else
  std::shared_ptr<ceres::LocalParameterization>
    auto_diff_manifold = make_local_parameterization_autodiff<SO2d>();
#endif

  // 0 + pi

  double x[2] = {1.0, 0.0};
  double delta[1] = {MANIF_PI};
  double x_plus_delta[2] = {0.0, 0.0};

  auto_diff_manifold->Plus(x, delta, x_plus_delta);

  EXPECT_DOUBLE_EQ(-1.0, x_plus_delta[0]);
  EXPECT_NEAR(0.0, x_plus_delta[1], 1e-15);

  EXPECT_EQ(MANIF_PI, Eigen::Map<const SO2d>(x_plus_delta).angle());

  // pi/4 + pi

  Eigen::Map<SO2d> map_so2(x);
  map_so2 = SO2d(MANIF_PI/4.);

//  delta[0] = MANIF_PI;
  x_plus_delta[0] = 0;
  x_plus_delta[1] = 0;

  auto_diff_manifold->Plus(x, delta, x_plus_delta);

  EXPECT_DOUBLE_EQ(cos(-3.*MANIF_PI/4.), x_plus_delta[0]);
  EXPECT_DOUBLE_EQ(sin(-3.*MANIF_PI/4.), x_plus_delta[1]);

  EXPECT_NEAR(-3.*MANIF_PI/4., Eigen::Map<const SO2d>(x_plus_delta).angle(), 1e-15);

  double J_rplus[2];
#if CERES_VERSION_MAJOR >= 2 && CERES_VERSION_MINOR >= 2
  auto_diff_manifold->PlusJacobian(x, J_rplus);
#else
  auto_diff_manifold->ComputeJacobian(x, J_rplus);
#endif

  EXPECT_DOUBLE_EQ(-0.70710678118654746, J_rplus[0]);
  EXPECT_DOUBLE_EQ( 0.70710678118654757, J_rplus[1]);
}

TEST(TEST_SO2_CERES, TEST_SO2_SMALL_PROBLEM_AUTODIFF)
{
  // Tell ceres not to take ownership of the raw pointers
  ceres::Problem::Options problem_options;
  problem_options.cost_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
#if CERES_VERSION_MAJOR >= 2 && CERES_VERSION_MINOR >= 2
  problem_options.manifold_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
#else
  problem_options.local_parameterization_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
#endif

  ceres::Problem problem(problem_options);

  // Create 4 objectives spread arround pi
  std::shared_ptr<ceres::CostFunction> obj_pi_over_4 =
      make_objective_autodiff<SO2d>(   MANIF_PI/4.);

  std::shared_ptr<ceres::CostFunction> obj_3_pi_over_8 =
      make_objective_autodiff<SO2d>(3.*MANIF_PI/8.);

  std::shared_ptr<ceres::CostFunction> obj_5_pi_over_8 =
      make_objective_autodiff<SO2d>(5.*MANIF_PI/8.);

  std::shared_ptr<ceres::CostFunction> obj_3_pi_over_4 =
      make_objective_autodiff<SO2d>(3.*MANIF_PI/4.);

  // Initializing state close to solution
  SO2d average_state( MANIF_PI/8.);

  /// @note Given the following init point,
  /// Ceres fails in computing derivatives (nans)
  /// when evaluating obj_3_pi_over_8 ...
//  SO2d average_state(3.*MANIF_PI/8.);

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

#if CERES_VERSION_MAJOR >= 2 && CERES_VERSION_MINOR >= 2
  std::shared_ptr<ceres::Manifold>
    auto_diff_manifold =
      make_manifold_autodiff<SO2d>();

  problem.SetManifold( average_state.data(),
                       auto_diff_manifold.get() );
#else
  std::shared_ptr<ceres::LocalParameterization>
    auto_diff_local_parameterization =
      make_local_parameterization_autodiff<SO2d>();

  problem.SetParameterization( average_state.data(),
                               auto_diff_local_parameterization.get() );
#endif

  std::cout << "-----------------------------\n";
  std::cout << "|       Calling Solve !     |\n";
  std::cout << "-----------------------------\n\n";

  // Run the solver!
  ceres::Solver::Options options;
  options.function_tolerance = 1e-15;
  options.minimizer_progress_to_stdout = true;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

//  std::cout << "summary:\n" << summary.BriefReport() << "\n";
  std::cout << "summary:\n" << summary.FullReport() << "\n";

  ASSERT_TRUE(summary.IsSolutionUsable());

  // 1.3088223838053636e-09
  EXPECT_ANGLE_NEAR(MANIF_PI_2, average_state.angle(), 1e-8);
}

TEST(TEST_SO2_CERES, TEST_SO2_CONSTRAINT_AUTODIFF)
{
  // Tell ceres not to take ownership of the raw pointers
  ceres::Problem::Options problem_options;
  problem_options.cost_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
#if CERES_VERSION_MAJOR >= 2 && CERES_VERSION_MINOR >= 2
  problem_options.manifold_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
#else
  problem_options.local_parameterization_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
#endif

  ceres::Problem problem(problem_options);

  GaussianNoiseGenerator<> noise(0, 0.1);

  //  p0 expected at  0
  //  p1 expected at  MANIF_PI/4.
  //  p2 expected at  MANIF_PI_2
  //  p3 expected at  3.*MANIF_PI/4.
  //  p4 expected at  MANIF_PI
  //  p5 expected at -3.*MANIF_PI/4
  //  p6 expected at -MANIF_PI_2
  //  p7 expected at -MANIF_PI/4.

  SO2d state_0( 0          + noise());
  SO2d state_1( MANIF_PI/4.    + noise());
  SO2d state_2( MANIF_PI_2     + noise());
  SO2d state_3( 3.*MANIF_PI/4. + noise());
  SO2d state_4( MANIF_PI       + noise());
  SO2d state_5(-3.*MANIF_PI/4  + noise());
  SO2d state_6(-MANIF_PI_2     + noise());
  SO2d state_7(-MANIF_PI/4.    + noise());

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

  auto constraint_0_1 = make_constraint_autodiff<SO2d>(MANIF_PI/4.);
  auto constraint_1_2 = make_constraint_autodiff<SO2d>(MANIF_PI/4.);
  auto constraint_2_3 = make_constraint_autodiff<SO2d>(MANIF_PI/4.);
  auto constraint_3_4 = make_constraint_autodiff<SO2d>(MANIF_PI/4.);
  auto constraint_4_5 = make_constraint_autodiff<SO2d>(MANIF_PI/4.);
  auto constraint_5_6 = make_constraint_autodiff<SO2d>(MANIF_PI/4.);
  auto constraint_6_7 = make_constraint_autodiff<SO2d>(MANIF_PI/4.);
  auto constraint_7_0 = make_constraint_autodiff<SO2d>(MANIF_PI/4.);

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

#if CERES_VERSION_MAJOR >= 2 && CERES_VERSION_MINOR >= 2
  std::shared_ptr<ceres::Manifold>
    auto_diff_manifold =
      make_manifold_autodiff<SO2d>();

  problem.SetManifold( state_0.data(),
                       auto_diff_manifold.get() );

  problem.SetManifold( state_1.data(),
                       auto_diff_manifold.get() );

  problem.SetManifold( state_2.data(),
                       auto_diff_manifold.get() );

  problem.SetManifold( state_3.data(),
                       auto_diff_manifold.get() );

  problem.SetManifold( state_4.data(),
                       auto_diff_manifold.get() );

  problem.SetManifold( state_5.data(),
                       auto_diff_manifold.get() );

  problem.SetManifold( state_6.data(),
                       auto_diff_manifold.get() );

  problem.SetManifold( state_7.data(),
                       auto_diff_manifold.get() );
#else
  std::shared_ptr<ceres::LocalParameterization>
    auto_diff_local_parameterization =
      make_local_parameterization_autodiff<SO2d>();

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
#endif

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
  EXPECT_ANGLE_NEAR(MANIF_PI/4.,    state_1.angle(), ceres_eps);
  EXPECT_ANGLE_NEAR(MANIF_PI_2,     state_2.angle(), ceres_eps);
  EXPECT_ANGLE_NEAR(3.*MANIF_PI/4., state_3.angle(), ceres_eps);
  EXPECT_ANGLE_NEAR(MANIF_PI,       state_4.angle(), ceres_eps);
  EXPECT_ANGLE_NEAR(-3.*MANIF_PI/4, state_5.angle(), ceres_eps);
  EXPECT_ANGLE_NEAR(-MANIF_PI_2,    state_6.angle(), ceres_eps);
  EXPECT_ANGLE_NEAR(-MANIF_PI/4.,   state_7.angle(), ceres_eps);
}

MANIF_TEST_JACOBIANS_CERES(SO2d);

MANIF_RUN_ALL_TEST;
