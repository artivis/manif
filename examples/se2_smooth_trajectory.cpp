#include "manif/SE2.h"
#include "manif/ceres/ceres.h"
#include "manif/algorithms/decasteljau.h"

#include <ceres/ceres.h>

#include <iostream>

namespace manif {

template <typename _Manifold>
class SmoothnessConstraint
    : public ceres::CostFunction
{
  using Manifold = _Manifold;
  using Tangent  = typename _Manifold::Tangent;
  using ManifoldJacobian = typename _Manifold::Jacobian;

  using JacobianMap = typename internal::traits_ceres<Manifold>::ConstraintJacobianMap;

  template <typename _Scalar>
  using ManifoldTemplate = typename _Manifold::template ManifoldTemplate<_Scalar>;

  template <typename _Scalar>
  using TangentTemplate = typename Tangent::template TangentTemplate<_Scalar>;

  static constexpr int DoF = Manifold::DoF;
  static constexpr int RepSize = Manifold::RepSize;

public:

  using Jacobian = typename internal::traits_ceres<Manifold>::ConstraintJacobian;

  template <typename... Args>
  SmoothnessConstraint(Args&&... args)
    : degree_(std::forward<Args>(args)...)
  {
    set_num_residuals(DoF);
    mutable_parameter_block_sizes()->push_back(Manifold::RepSize);
    mutable_parameter_block_sizes()->push_back(Manifold::RepSize);
    mutable_parameter_block_sizes()->push_back(Manifold::RepSize);
  }

  virtual ~SmoothnessConstraint() = default;

  template<typename T>
  bool operator()(const T* const initial_raw,
                  const T* const mid_raw,
                  const T* const final_raw,
                  T* residuals_raw) const
  {
    const Eigen::Map<const ManifoldTemplate<T>> initial(initial_raw);
    const Eigen::Map<const ManifoldTemplate<T>> mid(mid_raw);
    const Eigen::Map<const ManifoldTemplate<T>> final(final_raw);

    Eigen::Map<TangentTemplate<T>> residuals(residuals_raw);

    const ManifoldTemplate<T> r = final   + (TangentTemplate<T>::Zero()*T(t_));
    const ManifoldTemplate<T> l = initial + (TangentTemplate<T>::Zero()*T(t_));

    residuals = r + ((l - r) * T(psi_)) - mid;

    return true;
  }

  virtual bool Evaluate(double const* const* parameters_raw,
                        double* residuals_raw,
                        double** jacobians_raw) const
  {
    const Eigen::Map<const Manifold> state_past(parameters_raw[0]);
    const Eigen::Map<const Manifold> state_future(parameters_raw[1]);

    Eigen::Map<Tangent> residuals(residuals_raw);

    MANIF_THROW("NOP");
  }

protected:

  const double degree_;
  const double t_ = 0.5;
  const double psi_ = 0.5;
};

}

using namespace manif;

//using LocalParameterizationSE2 = LocalParameterization<SE2d>;
//using ObjectiveSE2 = Objective<SE2d>;
using SmoothnessConstraintSE2 = SmoothnessConstraint<SE2d>;

int main(int argc, char** argv)
{
  int degree = 2;

  if (argc >= 2)
  {
    degree = atoi(argv[1]);
  }

  // Tell ceres not to take ownership of the raw pointers
  ceres::Problem::Options problem_options;
  problem_options.cost_function_ownership          = ceres::DO_NOT_TAKE_OWNERSHIP;
  problem_options.local_parameterization_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;

  ceres::Problem problem(problem_options);

//  SE2 initial = SE2::Identity(),
//      final(4,15,M_PI);


  std::vector<SE2d> trajectory;

  for (int i=0; i<5; ++i)
    trajectory.emplace_back(0,0,0);

  std::vector<std::shared_ptr<SmoothnessConstraintSE2>> constraints;
  for (int i=1; i<trajectory.size()-1;++i)
  {
//    SmoothnessConstraintSE2 t( 2 );


    // Create the smoothness constraint
    constraints.push_back(
          std::make_shared<
                ceres::AutoDiffCostFunction<
                  SmoothnessConstraintSE2,
                  3,
                  4,
                  4,
                  4 >>( new SmoothnessConstraintSE2( 2 ) );
        );

    // Add constraint to Ceres

    problem.AddResidualBlock( constraints.back().get(),
                              nullptr,
                              trajectory[i-1].data(),
                              trajectory[ i ].data(),
                              trajectory[i+1].data() );

  }

  // Objective initial
  std::shared_ptr<ceres::CostFunction> obj_initial =
      make_objective_autodiff<SE2d>(0,0,0);

  problem.AddResidualBlock( obj_initial.get(),
                            nullptr,
                            trajectory[0].data() );

  // Objective final
  std::shared_ptr<ceres::CostFunction> obj_final =
      make_objective_autodiff<SE2d>(4,15,M_PI);

  problem.AddResidualBlock( obj_final.get(),
                            nullptr,
                            trajectory.back().data() );


  // Add autodiff local parametrization
  std::shared_ptr<ceres::LocalParameterization>
    auto_diff_local_parameterization =
      make_local_parametrization_autodiff<SE2d>();

  for (int i=0; i<trajectory.size(); ++i)
  {
    problem.SetParameterization( trajectory[i].data(),
                                 auto_diff_local_parameterization.get() );
  }

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

  std::cout << "Final states :\n";

  for (const auto& s : trajectory)
    std::cout << s.x() << "," << s.y() << "," << s.angle() << "\n";

  return 0;
}
