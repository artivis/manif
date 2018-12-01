#ifndef _MANIF_MANIF_CERES_TRAITS_H_
#define _MANIF_MANIF_CERES_TRAITS_H_

#include "manif/impl/lie_group_base.h"
#include "manif/impl/eigen.h"

namespace manif
{
namespace internal
{

template <typename _LieGroup>
struct traits_ceres
{
  using LieGroup = _LieGroup;

  using ObjectiveJacobian =
    Eigen::Matrix<typename LieGroup::Scalar,
                  LieGroup::DoF,
                  LieGroup::RepSize,
                  Eigen::RowMajor>;

  using ObjectiveJacobianMap =
    Eigen::Map<ObjectiveJacobian>;

  using ConstraintJacobian    = ObjectiveJacobian;
  using ConstraintJacobianMap = ObjectiveJacobianMap;

  using LocalParamJacobian =
    Eigen::Matrix<typename LieGroup::Scalar,
                  LieGroup::RepSize,
                  LieGroup::DoF,
                  (LieGroup::DoF>1)?
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
