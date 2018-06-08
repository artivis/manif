#ifndef _MANIF_MANIF_CERES_OBJECTIVE_H_
#define _MANIF_MANIF_CERES_OBJECTIVE_H_

#include "manif/ceres/ceres_traits.h"

#include <ceres/cost_function.h>

namespace manif
{

template <typename _Manifold>
class Objective : public ceres::CostFunction
{
  using Manifold = _Manifold;
  using Tangent  = typename _Manifold::Tangent;
  using ManifoldJacobian = typename _Manifold::Jacobian;

  using JacobianMap = typename internal::traits_ceres<Manifold>::ObjectiveJacobianMap;

  template <typename _Scalar>
  using ManifoldTemplate = typename _Manifold::template ManifoldTemplate<_Scalar>;

  template <typename _Scalar>
  using TangentTemplate = typename Tangent::template TangentTemplate<_Scalar>;

public:

  using Jacobian = typename internal::traits_ceres<Manifold>::ObjectiveJacobian;

  template <typename... Args>
  Objective(Args&&... args)
    : target_state_(std::forward<Args>(args)...)
  {
    set_num_residuals(Manifold::DoF);
    mutable_parameter_block_sizes()->push_back(Manifold::RepSize);
  }

  virtual ~Objective() = default;

  template <typename T>
  bool operator()(const T* const state_raw, T* residuals_raw) const
  {
    const Eigen::Map<const ManifoldTemplate<T>> state(state_raw);
    Eigen::Map<TangentTemplate<T>> error(residuals_raw);

    error = target_state_.template cast<T>() - state;

    return true;
  }

  virtual bool Evaluate(double const* const* parameters_raw,
                        double* residuals_raw,
                        double** jacobians_raw) const
  {
    const Eigen::Map<const Manifold> state(parameters_raw[0]);

    Eigen::Map<Tangent> error(residuals_raw);

    if (jacobians_raw != nullptr && jacobians_raw[0] != nullptr)
    {
      error = target_state_.rminus(state, Manifold::_, J_e_mb_);

      JacobianMap jacobian(jacobians_raw[0]);
      jacobian.template topLeftCorner<Manifold::DoF,Manifold::DoF>() = J_e_mb_;
      jacobian.template rightCols<Manifold::RepSize-Manifold::DoF>().setZero();
    }
    else
    {
      error = target_state_ - state;
    }

    return true;
  }

protected:

  const Manifold target_state_;
  mutable ManifoldJacobian J_e_mb_;
};

//using ObjectiveSO2 = Objective<SO2d>;
//using ObjectiveSO3 = Objective<SO3d>;
//using ObjectiveSE2 = Objective<SE2d>;
//using ObjectiveSE3 = Objective<SE3d>;

} /* namespace manif */

#endif /* _MANIF_MANIF_CERES_OBJECTIVE_H_ */
