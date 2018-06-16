#ifndef _MANIF_MANIF_CERES_CERES_H_
#define _MANIF_MANIF_CERES_CERES_H_

#include "manif/ceres/ceres_traits.h"
#include "manif/ceres/ceres_utils.h"
#include "manif/ceres/constraint.h"
#include "manif/ceres/local_parametrization.h"
#include "manif/ceres/objective.h"

namespace manif {
namespace detail {
struct YOU_MUST_INCLUDE_MANIF_BEFORE_CERES_HELPER_HEADERS{};
}

#ifdef _MANIF_MANIF_SO2_H_
using ConstraintSO2 = Constraint<SO2d>;
using LocalParameterizationSO2 = LocalParameterization<SO2d>;
using ObjectiveSO2 = Objective<SO2d>;
#else
using ConstraintSO2 = detail::YOU_MUST_INCLUDE_MANIF_BEFORE_CERES_HELPER_HEADERS;
using LocalParameterizationSO2 = detail::YOU_MUST_INCLUDE_MANIF_BEFORE_CERES_HELPER_HEADERS;
using ObjectiveSO2 = detail::YOU_MUST_INCLUDE_MANIF_BEFORE_CERES_HELPER_HEADERS;
#endif

#ifdef _MANIF_MANIF_SO3_H_
using ConstraintSO3 = Constraint<SO3d>;
using LocalParameterizationSO3 = LocalParameterization<SO3d>;
using ObjectiveSO3 = Objective<SO3d>;
#else
using ConstraintSO3 = detail::YOU_MUST_INCLUDE_MANIF_BEFORE_CERES_HELPER_HEADERS;
using LocalParameterizationSO3 = detail::YOU_MUST_INCLUDE_MANIF_BEFORE_CERES_HELPER_HEADERS;
using ObjectiveSO3 = detail::YOU_MUST_INCLUDE_MANIF_BEFORE_CERES_HELPER_HEADERS;
#endif

#ifdef _MANIF_MANIF_SE2_H_
using ConstraintSE2 = Constraint<SE2d>;
using LocalParameterizationSE2 = LocalParameterization<SE2d>;
using ObjectiveSE2 = Objective<SE2d>;
#else
using ConstraintSE2 = detail::YOU_MUST_INCLUDE_MANIF_BEFORE_CERES_HELPER_HEADERS;
using LocalParameterizationSE2 = detail::YOU_MUST_INCLUDE_MANIF_BEFORE_CERES_HELPER_HEADERS;
using ObjectiveSE2 = detail::YOU_MUST_INCLUDE_MANIF_BEFORE_CERES_HELPER_HEADERS;
#endif

#ifdef _MANIF_MANIF_SE3_H_
using ConstraintSE3 = Constraint<SE3d>;
using LocalParameterizationSE3 = LocalParameterization<SE3d>;
using ObjectiveSE3 = Objective<SE3d>;
#else
using ConstraintSE3 = detail::YOU_MUST_INCLUDE_MANIF_BEFORE_CERES_HELPER_HEADERS;
using LocalParameterizationSE3 = detail::YOU_MUST_INCLUDE_MANIF_BEFORE_CERES_HELPER_HEADERS;
using ObjectiveSE3= detail::YOU_MUST_INCLUDE_MANIF_BEFORE_CERES_HELPER_HEADERS;
#endif

} /* namespace manif */

#endif /* _MANIF_MANIF_CERES_CERES_H_ */
