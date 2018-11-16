#ifndef _MANIF_MANIF_CERES_UTILS_H_
#define _MANIF_MANIF_CERES_UTILS_H_

#include "manif/ceres/local_parametrization.h"
#include "manif/ceres/objective.h"
#include "manif/ceres/constraint.h"

#include <ceres/autodiff_local_parameterization.h>
#include <ceres/autodiff_cost_function.h>

namespace manif
{

/// @brief Specialize Constant traits
/// for the ceres::Jet type
template <typename _Scalar, int N>
struct Constants<ceres::Jet<_Scalar, N>>
{
  static const ceres::Jet<_Scalar, N> eps;
  static const ceres::Jet<_Scalar, N> eps_s;
//  static const Scalar eps_sqrt;

//  static const Scalar to_rad; // pi / 180
//  static const Scalar to_deg; // 180 / pi
};

template <typename _Scalar, int N>
const ceres::Jet<_Scalar, N>
Constants<ceres::Jet<_Scalar, N>>::eps = ceres::Jet<_Scalar, N>(1e-10);

template <typename _Scalar, int N>
const ceres::Jet<_Scalar, N>
Constants<ceres::Jet<_Scalar, N>>::eps_s = ceres::Jet<_Scalar, N>(1e-15);

//template <typename _Scalar, int N>
//const ceres::Jet<_Scalar, N>
//Constants<ceres::Jet<_Scalar, N>>::eps_sqrt = Constants<_Scalar>::eps_sqrt;

//template <typename _Scalar, int N>
//const ceres::Jet<_Scalar, N>
//Constants<ceres::Jet<_Scalar, N>>::to_deg = Constants<_Scalar>::to_deg;

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

template <typename _LieGroup>
std::shared_ptr<LocalParameterization<_LieGroup>>
make_local_parametrization()
{
  return std::make_shared<LocalParameterization<_LieGroup>>();
}

template <typename _LieGroup>
std::shared_ptr<
  ceres::AutoDiffLocalParameterization<LocalParameterization<_LieGroup>,
  _LieGroup::RepSize, _LieGroup::DoF>>
make_local_parametrization_autodiff()
{
  return std::make_shared<
      ceres::AutoDiffLocalParameterization<
        LocalParameterization<_LieGroup>, _LieGroup::RepSize, _LieGroup::DoF>>();
}

template <typename _LieGroup, typename... Args>
std::shared_ptr<Objective<_LieGroup>>
make_objective(Args&&... args)
{
  return std::make_shared<Objective<_LieGroup>>(std::forward<Args>(args)...);
}

template <typename _LieGroup, typename... Args>
std::shared_ptr<
  ceres::AutoDiffCostFunction<
    Objective<_LieGroup>, _LieGroup::DoF, _LieGroup::RepSize>>
make_objective_autodiff(Args&&... args)
{
  constexpr int DoF = _LieGroup::DoF;
  constexpr int RepSize = _LieGroup::RepSize;

  return std::make_shared<
      ceres::AutoDiffCostFunction<Objective<_LieGroup>, DoF, RepSize>>(
        new Objective<_LieGroup>(std::forward<Args>(args)...)
      );
}

/// @todo find a way to merge with make_objective_autodiff
template <typename _LieGroup, typename... Args>
std::shared_ptr<
  ceres::AutoDiffCostFunction<
    Objective<_LieGroup>, _LieGroup::DoF, _LieGroup::RepSize>>
make_weighted_objective_autodiff(const double weight, Args&&... args)
{
  constexpr int DoF = _LieGroup::DoF;
  constexpr int RepSize = _LieGroup::RepSize;

  Objective<_LieGroup>* obj = new Objective<_LieGroup>(std::forward<Args>(args)...);
  obj->weight(weight);

  return std::make_shared<
      ceres::AutoDiffCostFunction<Objective<_LieGroup>, DoF, RepSize>>(obj);
}

template <typename _LieGroup, typename... Args>
std::shared_ptr<Constraint<_LieGroup>>
make_constraint(Args&&... args)
{
  return std::make_shared<Constraint<_LieGroup>>(std::forward<Args>(args)...);
}

template <typename _LieGroup, typename... Args>
std::shared_ptr<
  ceres::AutoDiffCostFunction<
    Constraint<_LieGroup>, _LieGroup::DoF, _LieGroup::RepSize, _LieGroup::RepSize>>
make_constraint_autodiff(Args&&... args)
{
  return std::make_shared<
      ceres::AutoDiffCostFunction<
        Constraint<_LieGroup>,
        _LieGroup::DoF,
        _LieGroup::RepSize,
        _LieGroup::RepSize>>(
            new Constraint<_LieGroup>(std::forward<Args>(args)...));
}

} /* namespace manif */

#endif /* _MANIF_MANIF_CERES_UTILS_H_ */
