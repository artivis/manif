#ifndef _MANIF_MANIF_CERES_CONSTRAINT_H_
#define _MANIF_MANIF_CERES_CONSTRAINT_H_

#include "manif/ceres/ceres_traits.h"

#include <ceres/cost_function.h>

namespace manif
{

template <typename _Manifold>
class Constraint
    : public ceres::CostFunction
{
  using Manifold = _Manifold;
  using Tangent  = typename _Manifold::Tangent;
  using ManifoldJacobian = typename _Manifold::Jacobian;

  using JacobianMap = typename internal::traits_ceres<Manifold>::ConstraintJacobianMap;

  template <typename _Scalar>
  using ManifoldTemplate = typename manif::internal::traitscast<Manifold, _Scalar>::cast;

  template <typename _Scalar>
  using TangentTemplate = typename manif::internal::traitscast<Tangent, _Scalar>::cast;

  static constexpr int DoF = Manifold::DoF;
  static constexpr int RepSize = Manifold::RepSize;

public:

  using Jacobian = typename internal::traits_ceres<Manifold>::ConstraintJacobian;

  template <typename... Args>
  Constraint(Args&&... args)
    : measurement_(std::forward<Args>(args)...)
  {
    set_num_residuals(DoF);
    mutable_parameter_block_sizes()->push_back(Manifold::RepSize);
    mutable_parameter_block_sizes()->push_back(Manifold::RepSize);
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
          J_res_past.template topLeftCorner<DoF,DoF>().noalias() =
            J_res_pe_ * J_pe_pi_ * J_pi_past_;
          J_res_past.template rightCols<RepSize-DoF>().setZero();
        }

        if (jacobians_raw[1] != nullptr)
        {
          JacobianMap J_res_future(jacobians_raw[1]);
          J_res_future.template topLeftCorner<DoF,DoF>().noalias() =
            J_res_pe_ * J_pe_pi_ * J_pi_future_;
          J_res_future.template rightCols<RepSize-DoF>().setZero();
        }
      }
    }
    else
    {
      residuals =
        measurement_.retract()
          .between(state_past.between(state_future))
            .lift();

      /// @todo
//      residuals = measurement_ - (state_past - state_future);
    }

    return true;
  }

protected:

  const Tangent measurement_;

  mutable ManifoldJacobian J_pi_past_,  J_pi_future_;
  mutable ManifoldJacobian J_pe_pi_;
  mutable ManifoldJacobian J_res_pe_;
};

} /* namespace manif */

#endif /* _MANIF_MANIF_CERES_CONSTRAINT_H_ */
