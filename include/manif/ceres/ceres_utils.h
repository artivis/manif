#ifndef _MANIF_MANIF_CERES_UTILS_H_
#define _MANIF_MANIF_CERES_UTILS_H_

#if CERES_VERSION_MAJOR >= 2 && CERES_VERSION_MINOR >= 2
#include "manif/ceres/manifold.h"
#else
#include "manif/ceres/local_parametrization.h"
#endif
#include "manif/ceres/objective.h"
#include "manif/ceres/constraint.h"

#if CERES_VERSION_MAJOR >= 2 && CERES_VERSION_MINOR >= 2
#include <ceres/autodiff_manifold.h>
#else
#include <ceres/autodiff_local_parameterization.h>
#endif
#include <ceres/autodiff_cost_function.h>

namespace manif {

#if CERES_VERSION_MAJOR >= 2 && CERES_VERSION_MINOR >= 2
/**
 * @brief Helper function to create a Ceres Manifold parameterization wrapper.
 * @see CeresManifoldFunctor
 */
template <typename _LieGroup>
std::shared_ptr<
  ceres::AutoDiffManifold<CeresManifoldFunctor<_LieGroup>,
  _LieGroup::RepSize, _LieGroup::DoF>>
make_manifold_autodiff()
{
  return std::make_shared<
      ceres::AutoDiffManifold<
        CeresManifoldFunctor<_LieGroup>, _LieGroup::RepSize, _LieGroup::DoF>>();
}
#else
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
#endif

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
