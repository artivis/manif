#ifndef _MANIF_MANIF_CERES_LOCAL_PARAMETRIZATION_H_
#define _MANIF_MANIF_CERES_LOCAL_PARAMETRIZATION_H_

#include "manif/ceres/ceres_traits.h"

#include <ceres/local_parameterization.h>

namespace manif
{

template <typename _LieGroup>
class LocalParameterization
    : public ceres::LocalParameterization
{
  using LieGroup = _LieGroup;
  using Tangent  = typename _LieGroup::Tangent;

  using JacobianMap = typename internal::traits_ceres<LieGroup>::LocalParamJacobianMap;

  template <typename _Scalar>
  using LieGroupTemplate = typename manif::internal::traitscast<LieGroup, _Scalar>::cast;

  template <typename _Scalar>
  using TangentTemplate = typename manif::internal::traitscast<Tangent, _Scalar>::cast;

public:

  using Jacobian = typename internal::traits_ceres<LieGroup>::LocalParamJacobian;

  LocalParameterization() = default;
  virtual ~LocalParameterization() = default;

  template<typename T>
  bool operator()(const T* state_raw,
                  const T* delta_raw,
                  T* state_plus_delta_raw) const
  {
    const Eigen::Map<const LieGroupTemplate<T>> state(state_raw);
    const Eigen::Map<const TangentTemplate<T>>  delta(delta_raw);

    Eigen::Map<LieGroupTemplate<T>> state_plus_delta(state_plus_delta_raw);

    state_plus_delta = state + delta;

    return true;
  }

  /**
   * @brief Plus, rplus
   * @param state_raw
   * @param delta_raw
   * @param state_plus_delta_raw
   * @return true
   */
  virtual bool Plus(double const* state_raw,
                    double const* delta_raw,
                    double* state_plus_delta_raw) const override
  {
    return operator ()(state_raw, delta_raw, state_plus_delta_raw);
  }

  /**
   * @brief ComputeJacobian
   * @param T_raw
   * @param jacobian_raw
   * @return
   */
  virtual bool ComputeJacobian(double const* /*state_raw*/,
                               double* rplus_jacobian_raw) const override
  {
    JacobianMap(rplus_jacobian_raw).setIdentity();
    return true;
  }

  virtual int GlobalSize() const override { return LieGroup::RepSize; }
  virtual int LocalSize()  const override { return LieGroup::DoF; }
};

} /* namespace manif */

#endif /* _MANIF_MANIF_CERES_LOCAL_PARAMETRIZATION_H_ */
