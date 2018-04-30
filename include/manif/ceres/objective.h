#ifndef _MANIF_MANIF_CERES_OBJECTIVE_H_
#define _MANIF_MANIF_CERES_OBJECTIVE_H_

#include <ceres/cost_function.h>

namespace manif
{

template <typename _Manifold>
class Objective
    : public ceres::CostFunction
{
  using Manifold = _Manifold;
  using Tangent  = typename _Manifold::Tangent;
  using Jacobian = typename _Manifold::Jacobian;
  using JacobianMap =
    Eigen::Map<Eigen::Matrix<
      double, Manifold::DoF, Manifold::DoF, Eigen::RowMajor>>;

public:

  explicit Objective(const Manifold& target_state)
    : target_state_(target_state)
  {
    set_num_residuals(Manifold::DoF);
    mutable_parameter_block_sizes()->push_back(Manifold::RepSize);
  }

  virtual ~Objective() = default;

  virtual bool Evaluate(double const* const* parameters_raw,
                        double* residuals_raw,
                        double** jacobians_raw) const
  {
    const Eigen::Map<const Manifold> state(parameters_raw[0]);

    Eigen::Map<Tangent> error(residuals_raw);

    if (jacobians_raw != nullptr)
    {
      if (jacobians_raw[0] != nullptr)
      {
        target_state_.rminus(state, error, J_rminus_ma, J_rminus_mb);

        JacobianMap jacobian(jacobians_raw[0]);
        jacobian = J_rminus_mb;
      }
    }
    else
    {
      error = target_state_ - state;
    }

    return true;
  }

protected:

  const Manifold target_state_;
  mutable Jacobian J_rminus_ma, J_rminus_mb;
};

//using ObjectiveSO2 = Objective<SO2d>;
//using ObjectiveSO3 = Objective<SO3d>;
//using ObjectiveSE2 = Objective<SE2d>;
//using ObjectiveSE3 = Objective<SE3d>;

} /* namespace manif */

#endif /* _MANIF_MANIF_CERES_OBJECTIVE_H_ */
