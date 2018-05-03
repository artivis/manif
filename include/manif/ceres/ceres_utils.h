#ifndef _MANIF_MANIF_CERES_UTILS_H_
#define _MANIF_MANIF_CERES_UTILS_H_

#include "manif/ceres/local_parametrization.h"
#include "manif/ceres/objective.h"

#include <ceres/autodiff_local_parameterization.h>
#include <ceres/autodiff_cost_function.h>

namespace manif
{

std::string getReason(const ceres::TerminationType flag)
{
  switch(flag)
  {
    case ceres::CONVERGENCE:    return "CONVERGENCE";
    case ceres::NO_CONVERGENCE: return "NO_CONVERGENCE";
    case ceres::FAILURE:        return "FAILURE";
    case ceres::USER_SUCCESS:   return "USER_SUCCESS";
    case ceres::USER_FAILURE:   return "USER_FAILURE";
    default:                    return "UNKNOWN";
  }
}

template <typename _Manifold>
std::shared_ptr<ceres::LocalParameterization>
make_local_parametrization_autodiff()
{
  return std::make_shared<
      ceres::AutoDiffLocalParameterization<
        LocalParameterization<_Manifold>, _Manifold::RepSize, _Manifold::DoF>>();
}

template <typename _Manifold, typename... Args>
std::shared_ptr<ceres::CostFunction>
make_objective(Args&&... args)
{
  return std::make_shared<Objective<_Manifold>>(std::forward<Args>(args)...);
}

template <typename _Manifold, typename... Args>
std::shared_ptr<ceres::AutoDiffCostFunction<Objective<_Manifold>, _Manifold::DoF, _Manifold::RepSize>>
make_objective_autodiff(Args&&... args)
{
  constexpr int DoF = _Manifold::DoF;
  constexpr int RepSize = _Manifold::RepSize;

  return std::make_shared<ceres::AutoDiffCostFunction<Objective<_Manifold>, DoF, RepSize>>(
      new Objective<_Manifold>(_Manifold(std::forward<Args>(args)...))
        );
}

} /* namespace manif */

#endif /* _MANIF_MANIF_CERES_UTILS_H_ */
