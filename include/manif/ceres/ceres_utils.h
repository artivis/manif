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

template <typename _Manifold>
std::shared_ptr<LocalParameterization<_Manifold>>
make_local_parametrization()
{
  return std::make_shared<LocalParameterization<_Manifold>>();
}

template <typename _Manifold>
std::shared_ptr<
  ceres::AutoDiffLocalParameterization<LocalParameterization<_Manifold>,
  _Manifold::RepSize, _Manifold::DoF>>
make_local_parametrization_autodiff()
{
  return std::make_shared<
      ceres::AutoDiffLocalParameterization<
        LocalParameterization<_Manifold>, _Manifold::RepSize, _Manifold::DoF>>();
}

template <typename _Manifold, typename... Args>
std::shared_ptr<Objective<_Manifold>>
make_objective(Args&&... args)
{
  return std::make_shared<Objective<_Manifold>>(std::forward<Args>(args)...);
}

template <typename _Manifold, typename... Args>
std::shared_ptr<
  ceres::AutoDiffCostFunction<
    Objective<_Manifold>, _Manifold::DoF, _Manifold::RepSize>>
make_objective_autodiff(Args&&... args)
{
  constexpr int DoF = _Manifold::DoF;
  constexpr int RepSize = _Manifold::RepSize;

  return std::make_shared<
      ceres::AutoDiffCostFunction<Objective<_Manifold>, DoF, RepSize>>(
        new Objective<_Manifold>(_Manifold(std::forward<Args>(args)...))
      );
}

template <typename _Manifold, typename... Args>
std::shared_ptr<Constraint<_Manifold>>
make_constraint(Args&&... args)
{
  return std::make_shared<Constraint<_Manifold>>(std::forward<Args>(args)...);
}

template <typename _Manifold, typename... Args>
std::shared_ptr<
  ceres::AutoDiffCostFunction<
    Constraint<_Manifold>, _Manifold::DoF, _Manifold::RepSize, _Manifold::RepSize>>
make_constraint_autodiff(Args&&... args)
{
  return std::make_shared<
      ceres::AutoDiffCostFunction<
        Constraint<_Manifold>,
        _Manifold::DoF,
        _Manifold::RepSize,
        _Manifold::RepSize>>(
            new Constraint<_Manifold>(
             typename _Manifold::Tangent(std::forward<Args>(args)...)));
}

} /* namespace manif */

#endif /* _MANIF_MANIF_CERES_UTILS_H_ */
