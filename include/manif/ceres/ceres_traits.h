#ifndef _MANIF_MANIF_CERES_TRAITS_H_
#define _MANIF_MANIF_CERES_TRAITS_H_

#include "manif/impl/manifold_base.h"
#include "manif/impl/eigen.h"

namespace manif
{
namespace internal
{

template <typename _Manifold>
struct traits_ceres
{
  using Manifold = _Manifold;

  using ObjectiveJacobian =
    Eigen::Matrix<typename Manifold::Scalar,
                  Manifold::DoF,
                  Manifold::RepSize,
                  Eigen::RowMajor>;

  using ObjectiveJacobianMap =
    Eigen::Map<ObjectiveJacobian>;

  using ConstraintJacobian    = ObjectiveJacobian;
  using ConstraintJacobianMap = ObjectiveJacobianMap;

  using LocalParamJacobian =
    Eigen::Matrix<typename Manifold::Scalar,
                  Manifold::RepSize,
                  Manifold::DoF,
                  (Manifold::DoF>1)?
                    Eigen::RowMajor:
                    Eigen::ColMajor>;

  /// @note Without the ternary op
  /// error: static assertion failed: INVALID_MATRIX_TEMPLATE_PARAMETERS
  /// EIGEN_STATIC_ASSERT((EIGEN_IMPLIES(MaxRowsAtCompileTime==1 && MaxColsAtCompileTime!=1, (Options&RowMajor)==RowMajor)

  using LocalParamJacobianMap =
    Eigen::Map<LocalParamJacobian>;
};

} /* namespace internal */
} /* namespace manif */

#endif /* _MANIF_MANIF_CERES_TRAITS_H_ */
