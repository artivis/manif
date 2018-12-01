#ifndef _MANIF_MANIF_CERES_OBJECTIVE_H_
#define _MANIF_MANIF_CERES_OBJECTIVE_H_

#include "manif/ceres/ceres_traits.h"

#include <memory>
#include <ceres/cost_function.h>

namespace manif
{

template <typename _LieGroup>
class Objective : public ceres::CostFunction
{
  using LieGroup = _LieGroup;
  using Tangent  = typename _LieGroup::Tangent;
  using LieGroupJacobian = typename _LieGroup::Jacobian;

  using JacobianMap = typename internal::traits_ceres<LieGroup>::ObjectiveJacobianMap;

  template <typename _Scalar>
  using LieGroupTemplate = typename manif::internal::traitscast<LieGroup, _Scalar>::cast;

  template <typename _Scalar>
  using TangentTemplate = typename manif::internal::traitscast<Tangent, _Scalar>::cast;

public:

  using Jacobian = typename internal::traits_ceres<LieGroup>::ObjectiveJacobian;

  template <typename... Args>
  Objective(Args&&... args)
    : target_state_(std::forward<Args>(args)...)
  {
    set_num_residuals(LieGroup::DoF);
    mutable_parameter_block_sizes()->push_back(LieGroup::RepSize);
  }

  virtual ~Objective() = default;

  template <typename T>
  bool operator()(const T* const state_raw, T* residuals_raw) const
  {
    const Eigen::Map<const LieGroupTemplate<T>> state(state_raw);
    Eigen::Map<TangentTemplate<T>> error(residuals_raw);

    error = (target_state_.template cast<T>() - state) * T(weight_);

    return true;
  }

  virtual bool Evaluate(double const* const* parameters_raw,
                        double* residuals_raw,
                        double** jacobians_raw) const
  {
    const Eigen::Map<const LieGroup> state(parameters_raw[0]);

    Eigen::Map<Tangent> error(residuals_raw);

    if (jacobians_raw != nullptr && jacobians_raw[0] != nullptr)
    {
      error = target_state_.rminus(state, LieGroup::_, J_e_mb_);

      JacobianMap jacobian(jacobians_raw[0]);
      jacobian.template topLeftCorner<LieGroup::DoF,LieGroup::DoF>() = J_e_mb_;
      jacobian.template rightCols<LieGroup::RepSize-LieGroup::DoF>().setZero();
    }
    else
    {
      error = target_state_ - state;
    }

    return true;
  }

  inline void weight(const double weight) { weight_ = weight; }
  inline double weight() const noexcept { return weight_; }

protected:

  double weight_ = 1;
  const LieGroup target_state_;
  mutable LieGroupJacobian J_e_mb_;
};

template <typename _LieGroup>
using ObjectivePtr = std::shared_ptr<Objective<_LieGroup>>;

} /* namespace manif */

#endif /* _MANIF_MANIF_CERES_OBJECTIVE_H_ */
