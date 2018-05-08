#ifndef _MANIF_MANIF_CERES_CONSTRAINT_H_
#define _MANIF_MANIF_CERES_CONSTRAINT_H_

#include "manif/ceres/ceres_jacobian_helper.h"

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

  using JacobianMap = typename internal::traits_ceres<Manifold>::JacobianMap;

  template <typename _Scalar>
  using ManifoldTemplate = typename _Manifold::template ManifoldTemplate<_Scalar>;

  template <typename _Scalar>
  using TangentTemplate = typename Tangent::template TangentTemplate<_Scalar>;

public:

  template <typename... Args>
  Constraint(Args&&... args)
    : measurement_(std::forward<Args>(args)...)
  {
    constexpr int DoF = Manifold::DoF;
    constexpr int RepSize = Manifold::RepSize;

    set_num_residuals(DoF);
    mutable_parameter_block_sizes()->push_back(RepSize);
    mutable_parameter_block_sizes()->push_back(RepSize);
  }

  virtual ~Constraint() = default;

  template<typename T>
  bool operator()(const T* const past_raw,
                  const T* const futur_raw,
                  T* residuals_raw) const
  {
    const Eigen::Map<const ManifoldTemplate<T>> state_past(past_raw);
    const Eigen::Map<const ManifoldTemplate<T>> state_future(futur_raw);

    Eigen::Map<TangentTemplate<T>> residuals(residuals_raw);

    residuals =
      measurement_.retract().template cast<T>()
        .between(state_past.between(state_future))
          .lift();

    return true;
  }

  virtual bool Evaluate(double const* const* parameters_raw,
                        double* residuals_raw,
                        double** jacobians_raw) const
  {
    const Eigen::Map<const Manifold> state_past(parameters_raw[0]);
    const Eigen::Map<const Manifold> state_future(parameters_raw[1]);

    Eigen::Map<Tangent> residuals(residuals_raw);

    if (jacobians_raw != nullptr)
    {
      if (jacobians_raw[0] != nullptr ||
          jacobians_raw[1] != nullptr)
      {
        typename Manifold::OptJacobianRef J_pi_past;
        typename Manifold::OptJacobianRef J_pi_future;

        if (jacobians_raw[0] != nullptr)
        {
          J_pi_past = J_pi_past_;
        }

        if (jacobians_raw[1] != nullptr)
        {
          J_pi_future = J_pi_future_;
        }

        residuals = measurement_.retract().
                      between( state_past.between(state_future,
                                J_pi_past, J_pi_future) ,      Manifold::_, J_pe_pi_).
                        lift(J_res_pe_);

        if (jacobians_raw[0] != nullptr)
        {
          JacobianMap J_res_past(jacobians_raw[0]);
          J_res_past =
            computeLiftJacobianGlobal(state_past) * J_res_pe_ * J_pe_pi_ * J_pi_past_;
        }

        if (jacobians_raw[1] != nullptr)
        {
          JacobianMap J_res_future(jacobians_raw[1]);
          J_res_future =
            computeLiftJacobianGlobal(state_future) * J_res_pe_ * J_pe_pi_ * J_pi_future_;
        }
      }
    }
    else
    {
      residuals =
        measurement_.retract()
          .between(state_past.between(state_future))
            .lift();
    }

    return true;
  }

protected:

  const Tangent measurement_;

  mutable Jacobian J_pi_past_,  J_pi_future_;
  mutable Jacobian J_pe_pi_;
  mutable Jacobian J_res_pe_;
};

//using ConstraintSO2 = Constraint<SO2d>;
//using ConstraintSO3 = Constraint<SO3d>;
//using ConstraintSE2 = Constraint<SE2d>;
//using ConstraintSE3 = Constraint<SE3d>;

} /* namespace manif */

#endif /* _MANIF_MANIF_CERES_CONSTRAINT_H_ */
