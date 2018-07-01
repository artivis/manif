#include "manif/SE2.h"
#include "manif/ceres/ceres.h"
#include "manif/algorithms/interpolation.h"

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
    mutable_parameter_block_sizes()->push_back(1);
  }

  virtual ~SmoothnessConstraint() = default;

  template<typename T>
  bool operator()(const T* const initial_raw,
                  const T* const mid_raw,
                  const T* const final_raw,
                  const T* const initial_stamp,
                  const T* const mid_stamp,
                  const T* const final_stamp,
                  T* residuals_raw) const
  {
    const Eigen::Map<const ManifoldTemplate<T>> initial(initial_raw);
    const Eigen::Map<const ManifoldTemplate<T>> mid(mid_raw);
    const Eigen::Map<const ManifoldTemplate<T>> final(final_raw);

    Eigen::Map<TangentTemplate<T>> residuals(residuals_raw);

    const T stamp = ((*mid_stamp)   - (*initial_stamp)) /
                    ((*final_stamp) - (*initial_stamp));

//    std::cout << "initial_stamp : " << (*initial_stamp) << "\n";
//    std::cout << "mid_stamp : " << (*mid_stamp) << "\n";
//    std::cout << "final_stamp : " << (*final_stamp) << "\n";
//    std::cout << "stamp : " << stamp << "\n";

    const ManifoldTemplate<T> r = final   + (TangentTemplate<T>::Zero()*stamp);
    const ManifoldTemplate<T> l = initial + (TangentTemplate<T>::Zero()*stamp);

    const T phi = smoothing_phi(stamp, degree_);

    residuals = r + ((l - r) * T(phi)) - mid;

    return true;
  }

  virtual bool Evaluate(double const* const* parameters_raw,
                        double* residuals_raw,
                        double** jacobians_raw) const
  {
    MANIF_THROW("NOP");

//    const Eigen::Map<const Manifold> state_past(parameters_raw[0]);
//    const Eigen::Map<const Manifold> state_future(parameters_raw[1]);

//    Eigen::Map<Tangent> residuals(residuals_raw);

    return true;

  }

protected:

  const std::size_t degree_;
};

class ConstraintMinimizeScalar
    : public ceres::SizedCostFunction<1,1>
{
public:

  ConstraintMinimizeScalar() = default;
  ConstraintMinimizeScalar(const double w) : w_(w) { }
  virtual ~ConstraintMinimizeScalar() = default;

  template<typename T>
  bool operator()(const T* const scalar_raw,
                  T* residuals_raw) const
  {
    residuals_raw[0] = - w_ * scalar_raw[0];
    return true;
  }

  virtual bool Evaluate(double const* const* parameters_raw,
                        double* residuals_raw,
                        double** jacobians_raw) const
  {
    assert(parameters_raw    != nullptr);
    assert(parameters_raw[0] != nullptr);

    operator ()(parameters_raw[0], residuals_raw);

    if (jacobians_raw != nullptr && jacobians_raw[0] != nullptr)
    {
      jacobians_raw[0][0] = -w_;
    }

    return true;
  }

protected:

  double w_ = 1;
};

template <typename T>
inline T intervalCost(const T& var,const T& a,
                      const T& b, const T& epsilon)
{
  return (var < a+epsilon)  ? (-var + (a + epsilon)) :
         (var <= b-epsilon) ? T(0)                   :
                              (var - (b - epsilon));
}

template <typename _Manifold>
class VelocityConstraint
    : public ceres::CostFunction
{
  using Scalar = typename _Manifold::Scalar;
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

  using Velocity = Eigen::Matrix<Scalar, DoF, 1>;

  template <typename... Args>
  VelocityConstraint(const Velocity& lower_bound,
                     const Velocity& upper_bound)
    : lower_bound_(lower_bound)
    , upper_bound_(upper_bound)
  {
    set_num_residuals(DoF);
    mutable_parameter_block_sizes()->push_back(Manifold::RepSize);
    mutable_parameter_block_sizes()->push_back(Manifold::RepSize);
    mutable_parameter_block_sizes()->push_back(1);
    mutable_parameter_block_sizes()->push_back(1);
  }

  virtual ~VelocityConstraint() = default;

  template<typename T>
  bool operator()(const T* const past_raw,
                  const T* const future_raw,
                  const T* const past_stamp,
                  const T* const future_stamp,
                  T* residuals_raw) const
  {
    const Eigen::Map<const ManifoldTemplate<T>> past(past_raw);
    const Eigen::Map<const ManifoldTemplate<T>> future(future_raw);

    Eigen::Map<TangentTemplate<T>> residuals(residuals_raw);

    /// div by zero
    /// @todo prevent it
    if ((*future_stamp) == (*past_stamp))
      return false;

    T dt = (*future_stamp) - (*past_stamp);

    const TangentTemplate<T> vel = (future - past) * (T(1) / dt);

//    std::cout << "future stamp : " << (*future_stamp) << "\n";
//    std::cout << "past stamp : " << (*past_stamp) << "\n";
//    std::cout << "dt : " << dt << "\n";
//    std::cout << "vel : " << vel.coeffs()[0] << ","
//                          << vel.coeffs()[1] << ","
//                          << vel.coeffs()[2] << "\n";
//    std::cout << "lower_bound : " << lower_bound_.transpose() << "\n";
//    std::cout << "upper_bound : " << upper_bound_.transpose() << "\n\n";

    for (int i=0; i<DoF; ++i)
    {
      residuals.coeffs()[i] = intervalCost(vel.coeffs()[i],
                                           T(lower_bound_[i]),
                                           T(upper_bound_[i]),
                                           T(1e-5));
    }

    return true;
  }

  virtual bool Evaluate(double const* const* parameters_raw,
                        double* residuals_raw,
                        double** jacobians_raw) const
  {
    MANIF_THROW("NOP");
    return true;
  }

protected:

  const Velocity lower_bound_;
  const Velocity upper_bound_;
};

}

using namespace manif;

using SmoothnessConstraintSE2 = SmoothnessConstraint<SE2d>;
using VelocityConstraintSE2   = VelocityConstraint<SE2d>;

void use()
{
  std::cout << "./se2_smooth_trajectory <degree> <num_pts> <x,y,theta>\n\n";
}

int main(int argc, char** argv)
{
  int degree = 2;

  if (argc >= 2)
  {
    degree = atoi(argv[1]);
  }

  int n_pts = 5;

  if (argc >= 3)
  {
    n_pts = atoi(argv[2]);
  }

  double final_x = 4,
         final_y = 15,
         final_t = M_PI;

  if (argc == 6)
  {
    final_x = atof(argv[3]);
    final_y = atof(argv[4]);
    final_t = atof(argv[5]);
  }
  else if (argc > 3)
  {
    use();
    return EXIT_FAILURE;
  }

  std::cout << 2 << "," << n_pts << "," << degree << "\n";

  // Tell ceres not to take ownership of the raw pointers
  ceres::Problem::Options problem_options;
  problem_options.cost_function_ownership          = ceres::DO_NOT_TAKE_OWNERSHIP;
  problem_options.local_parameterization_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;

  ceres::Problem problem(problem_options);

  std::vector<SE2d> trajectory;

  Eigen::Matrix<double, Eigen::Dynamic, 1> stamps =
      Eigen::Matrix<double, Eigen::Dynamic, 1>::Zero(n_pts);

  for (int i=0; i<n_pts; ++i)
  {
    trajectory.emplace_back(0,0,0);
    stamps[i] = i;
  }

  // Force T_0 == 0
  std::shared_ptr<ceres::CostFunction> T0_constraint = std::make_shared< ConstraintMinimizeScalar >();

  problem.AddResidualBlock( T0_constraint.get(),
                            nullptr,
                            &stamps[0]          ) ;

  // Smoothness constraints
  std::vector<std::shared_ptr<ceres::CostFunction>> smoothness_constraints;
  for (int i=1; i<trajectory.size()-1;++i)
  {
    // Create the smoothness constraint
    smoothness_constraints.push_back(
          std::make_shared<
                ceres::AutoDiffCostFunction<
                  SmoothnessConstraintSE2,
                  SE2d::DoF,
                  SE2d::RepSize,
                  SE2d::RepSize,
                  SE2d::RepSize,
                  1,1,1> >( new SmoothnessConstraintSE2( degree ) )
        );

    // Add constraint to Ceres
    problem.AddResidualBlock( smoothness_constraints.back().get(),
                              nullptr,
                              trajectory[i-1].data(),
                              trajectory[ i ].data(),
                              trajectory[i+1].data(),
                              &stamps[i-1],
                              &stamps[ i ],
                              &stamps[i+1]);
  }

  // Velocity constraints
  const Eigen::Vector3d velocity_lower_bound(-0.5,-0.1,-0.4);
  const Eigen::Vector3d velocity_upper_bound( 1,   0.1, 0.8);

  std::vector<std::shared_ptr<ceres::CostFunction>> velocity_constraints;
  for (int i=0; i<trajectory.size()-1; ++i)
  {
    // Create the smoothness constraint
    velocity_constraints.push_back(
          std::make_shared<
                ceres::AutoDiffCostFunction<
                  VelocityConstraintSE2,
                  SE2d::DoF,
                  SE2d::RepSize,
                  SE2d::RepSize,
                  1, 1> >( new VelocityConstraintSE2( velocity_lower_bound,
                                                      velocity_upper_bound) )
        );

    // Add constraint to Ceres
    problem.AddResidualBlock( velocity_constraints.back().get(),
                              nullptr,
                              trajectory[ i ].data(),
                              trajectory[i+1].data(),
                              &stamps[ i ],
                              &stamps[i+1]);
  }

  // Time constraints
//  std::vector<std::shared_ptr<ceres::CostFunction>> time_constraints;
//  for (int i=0; i<trajectory.size();++i)
//  {
//    // Create time constraints
//    time_constraints.push_back(
//          std::make_shared< ConstraintMinimizeScalar >()
//        );

//    // Minimize time
//    problem.AddResidualBlock( time_constraints.back().get(),
//                              nullptr,
//                              &stamps[i]              );
//  }

  // Objective initial
  std::shared_ptr<ceres::CostFunction> obj_initial =
      make_objective_autodiff<SE2d>(0,0,0);

  problem.AddResidualBlock( obj_initial.get(),
                            nullptr,
                            trajectory[0].data() );

  // Objective final
  std::shared_ptr<ceres::CostFunction> obj_final =
      make_objective_autodiff<SE2d>(final_x,final_y,final_t);

  problem.AddResidualBlock( obj_final.get(),
                            nullptr,
                            trajectory.back().data() );

  std::cout << 0 << "," << 0 << "," << 0 << "\n";
  std::cout << final_x << "," << final_y << "," << final_t << "\n";

  // Add autodiff local parametrization
  std::shared_ptr<ceres::LocalParameterization>
    auto_diff_local_parameterization =
      make_local_parametrization_autodiff<SE2d>();

  for (int i=0; i<trajectory.size(); ++i)
  {
    problem.SetParameterization( trajectory[i].data(),
                                 auto_diff_local_parameterization.get() );
  }

//  std::cout << "-----------------------------\n";
//  std::cout << "|       Calling Solve !     |\n";
//  std::cout << "-----------------------------\n\n";

  // Run the solver!
  ceres::Solver::Options options;
  options.function_tolerance = 1e-15;
  options.minimizer_progress_to_stdout = false;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

//  std::cout << "summary:\n" << summary.BriefReport() << "\n\n";
//  std::cout << "summary:\n" << summary.FullReport() << "\n";

//  std::cout << "Final states :\n\n";

  for (const auto& s : trajectory)
    std::cout << s.x() << "," << s.y() << "," << s.angle() << "\n";

  std::cout << stamps << "\n";

//  for (const auto& t : pose_stamps)
//    std::cout << t << "\n";

  return 0;
}
