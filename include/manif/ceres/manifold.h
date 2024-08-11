#ifndef _MANIF_MANIF_CERES_MANIFOLD_H_
#define _MANIF_MANIF_CERES_MANIFOLD_H_

#include <Eigen/Core>

namespace manif {

/**
 * @brief A wrapper for Ceres autodiff local parameterization.
 */
template <typename _LieGroup>
class CeresManifoldFunctor
{
  using LieGroup = _LieGroup;
  using Tangent  = typename _LieGroup::Tangent;

  template <typename _Scalar>
  using LieGroupTemplate = typename LieGroup::template LieGroupTemplate<_Scalar>;

  template <typename _Scalar>
  using TangentTemplate = typename Tangent::template TangentTemplate<_Scalar>;

public:

  CeresManifoldFunctor() = default;
  virtual ~CeresManifoldFunctor() = default;

  template <typename T>
  bool Plus(const T* state_raw,
            const T* delta_raw,
            T* state_plus_delta_raw) const
  {
    const Eigen::Map<const LieGroupTemplate<T>> state(state_raw);
    const Eigen::Map<const TangentTemplate<T>>  delta(delta_raw);

    Eigen::Map<LieGroupTemplate<T>> state_plus_delta(state_plus_delta_raw);

    state_plus_delta = state + delta;

    return true;
  }

  template <typename T>
  bool Minus(const T* y_raw,
             const T* x_raw,
             T* y_minus_x_raw) const
  {
    const Eigen::Map<const LieGroupTemplate<T>> y(y_raw);
    const Eigen::Map<const LieGroupTemplate<T>>  x(x_raw);

    Eigen::Map<TangentTemplate<T>> y_minus_x(y_minus_x_raw);

    y_minus_x = y - x;

    return true;
  }
};

} /* namespace manif */

#endif /* _MANIF_MANIF_CERES_MANIFOLD_H_ */
