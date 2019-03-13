#ifndef _MANIF_MANIF_CERES_UTILS_H_
#define _MANIF_MANIF_CERES_UTILS_H_

#include "manif/ceres/local_parametrization.h"
#include "manif/ceres/objective.h"
#include "manif/ceres/constraint.h"

#include <ceres/autodiff_local_parameterization.h>
#include <ceres/autodiff_cost_function.h>

namespace manif {

/**
 * @brief Helper function to create a Ceres autodiff local parameterization wrapper.
 * @see CeresLocalParameterizationFunctor
 */
template <typename _LieGroup>
std::shared_ptr<
  ceres::AutoDiffLocalParameterization<CeresLocalParameterizationFunctor<_LieGroup>,
  _LieGroup::RepSize, _LieGroup::DoF>>
make_local_parameterization_autodiff()
{
  return std::make_shared<
      ceres::AutoDiffLocalParameterization<
        CeresLocalParameterizationFunctor<_LieGroup>, _LieGroup::RepSize, _LieGroup::DoF>>();
}

/**
 * @brief Helper function to create a Ceres autodiff objective wrapper.
 * @see CeresObjectiveFunctor
 */
template <typename _LieGroup, typename... Args>
std::shared_ptr<
  ceres::AutoDiffCostFunction<
    CeresObjectiveFunctor<_LieGroup>, 1, _LieGroup::RepSize>>
make_objective_autodiff(Args&&... args)
{
  return std::make_shared<
      ceres::AutoDiffCostFunction<CeresObjectiveFunctor<_LieGroup>, 1, _LieGroup::RepSize>>(
        new CeresObjectiveFunctor<_LieGroup>(std::forward<Args>(args)...)
      );
}

/**
 * @brief Helper function to create a Ceres autodiff constraint wrapper.
 * @see CeresConstraintFunctor
 */
template <typename _LieGroup, typename... Args>
std::shared_ptr<
  ceres::AutoDiffCostFunction<
    CeresConstraintFunctor<_LieGroup>, _LieGroup::DoF, _LieGroup::RepSize, _LieGroup::RepSize>>
make_constraint_autodiff(Args&&... args)
{
  return std::make_shared<
      ceres::AutoDiffCostFunction<
        CeresConstraintFunctor<_LieGroup>,
        _LieGroup::DoF,
        _LieGroup::RepSize,
        _LieGroup::RepSize>>(
            new CeresConstraintFunctor<_LieGroup>(std::forward<Args>(args)...));
}

} /* namespace manif */

#endif /* _MANIF_MANIF_CERES_UTILS_H_ */
