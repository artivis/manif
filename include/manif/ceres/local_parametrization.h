#ifndef _MANIF_MANIF_CERES_LOCAL_PARAMETRIZATION_H_
#define _MANIF_MANIF_CERES_LOCAL_PARAMETRIZATION_H_

#include <Eigen/Core>

namespace manif {

/**
 * @brief A wrapper for Ceres autodiff local parameterization.
 */
template <typename _LieGroup>
class CeresLocalParameterizationFunctor
{
  using LieGroup = _LieGroup;
  using Tangent  = typename _LieGroup::Tangent;

  template <typename _Scalar>
  using LieGroupTemplate = typename LieGroup::template LieGroupTemplate<_Scalar>;

  template <typename _Scalar>
  using TangentTemplate = typename Tangent::template TangentTemplate<_Scalar>;

public:

  CeresLocalParameterizationFunctor() = default;
  virtual ~CeresLocalParameterizationFunctor() = default;

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
};

} /* namespace manif */

#endif /* _MANIF_MANIF_CERES_LOCAL_PARAMETRIZATION_H_ */
