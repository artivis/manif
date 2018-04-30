#include <gtest/gtest.h>

#include "manif/SO2.h"
#include "manif/ceres/local_parametrization.h"
#include "manif/ceres/objective.h"

#include <ceres/ceres.h>

namespace manif
{

using LocalParameterizationSO2 = LocalParameterization<SO2d>;
using ObjectiveSO2 = Objective<SO2d>;

} /* namespace manif */

using namespace manif;

std::string getReason(const int flag)
{
  switch(flag)
  {
    case 0: return "DID_NOT_RUN";
    case 1: return "NO_CONVERGENCE";
    case 5: return "NUMERICAL_FAILURE";
    default: return "UNKNOWN";
  }
}

TEST(TEST_LOCAL_PARAMETRIZATION, TEST_SO2)
{
  // Tell ceres not to take ownership of the raw pointers
  ceres::Problem::Options problem_options;
  problem_options.cost_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;

  ceres::Problem problem(problem_options);

  SO2d average_state(0);

  // Create 4 objectives spread arround pi
  ObjectiveSO2 obj_pi_over_4(SO2d(M_PI/4.)),
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

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
