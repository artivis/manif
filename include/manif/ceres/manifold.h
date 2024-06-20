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

  bool Plus(const double* state_raw,
            const double* delta_raw,
            double* state_plus_delta_raw) const
  {
    const Eigen::Map<const LieGroupTemplate<T>> state(state_raw);
    const Eigen::Map<const TangentTemplate<T>>  delta(delta_raw);

    Eigen::Map<LieGroupTemplate<T>> state_plus_delta(state_plus_delta_raw);

    state_plus_delta = state + delta;

    return true;
  }

  bool Minus(const double* y_raw,
             const double* x_raw,
             double* y_minus_x_raw) const
  {
    const Eigen::Map<const LieGroupTemplate<T>> y(y_raw);
    const Eigen::Map<const TangentTemplate<T>>  x(x_raw);

    Eigen::Map<LieGroupTemplate<T>> y_minus_x(y_minus_x_raw);

    y_minus_x = y - x;

    return true;
  }
};

} /* namespace manif */

#endif /* _MANIF_MANIF_CERES_MANIFOLD_H_ */
