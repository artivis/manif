#ifndef _MANIF_MANIF_CERES_CONSTRAINT_H_
#define _MANIF_MANIF_CERES_CONSTRAINT_H_

#include <ceres/cost_function.h>

namespace manif
{

template <typename _Manifold>
class Constraint
    : public ceres::CostFunction
{
  using Manifold = _Manifold;
  using Tangent  = typename _Manifold::Tangent;
  using Jacobian = typename _Manifold::Jacobian;
  using JacobianMap =
    Eigen::Map<Eigen::Matrix<
      double, Manifold::DoF, Manifold::DoF, Eigen::RowMajor>>;

public:

  explicit Constraint(const Tangent& measurement)
    : measurement_(measurement)
  {
    set_num_residuals(Manifold::DoF);
    mutable_parameter_block_sizes()->push_back(Manifold::RepSize);
    mutable_parameter_block_sizes()->push_back(Manifold::RepSize);
  }

  virtual ~Constraint() = default;

  virtual bool Evaluate(double const* const* parameters_raw,
                        double* residuals_raw,
                        double** jacobians_raw) const
  {
    const Eigen::Map<const Manifold> state_past(parameters_raw[0]);
    const Eigen::Map<const Manifold> state_future(parameters_raw[1]);

    Eigen::Map<Tangent> residuals(residuals_raw);

    if (jacobians_raw != nullptr)
    {
      if (jacobians_raw[0] != nullptr &&
          jacobians_raw[1] != nullptr)
      {
        state_future.between(state_past,
                             pose_increment_,
                             J_pi_future_, J_pi_past_);

        measurement_.retract(mmeas_, J_mmeas_meas_);

        mmeas_.between(pose_increment_,
                       pe_,
                       J_pe_mmeas_, J_pe_pi_);

        pe_.lift(residuals, J_res_pe_);

        JacobianMap J_res_past(jacobians_raw[0]);
        JacobianMap J_res_future(jacobians_raw[1]);

        J_res_past   = J_res_pe_ * J_pe_pi_ * J_pi_past_;
        J_res_future = J_res_pe_ * J_pe_pi_ * J_pi_future_;
      }
    }
    else
    {
      residuals =
        measurement_.retract()
          .between(state_future.between(state_past))
            .lift();
    }

    return true;
  }

protected:

  const Tangent measurement_;

  mutable Manifold mmeas_;
  mutable Jacobian J_mmeas_meas_;

  mutable Manifold pose_increment_;
  mutable Jacobian J_pi_past_, J_pi_future_;

  mutable Jacobian J_pe_mmeas_, J_pe_pi_;

  mutable Manifold pe_;
  mutable Jacobian J_res_pe_;
};

//using ConstraintSO2 = Constraint<SO2d>;
//using ConstraintSO3 = Constraint<SO3d>;
//using ConstraintSE2 = Constraint<SE2d>;
//using ConstraintSE3 = Constraint<SE3d>;

} /* namespace manif */

#endif /* _MANIF_MANIF_CERES_CONSTRAINT_H_ */
