#ifndef _MANIF_MANIF_CERES_CERES_H_
#define _MANIF_MANIF_CERES_CERES_H_

#include "manif/ceres/constants.h"
#include "manif/ceres/constraint.h"
#if CERES_VERSION_MAJOR >= 2 && CERES_VERSION_MINOR >= 2
#include "manif/ceres/manifold.h"
#else
#include "manif/ceres/local_parametrization.h"
#endif
#include "manif/ceres/objective.h"
#include "manif/ceres/ceres_utils.h"

namespace manif {
namespace internal {
struct YOU_MUST_INCLUDE_MANIF_BEFORE_CERES_HELPER_HEADERS{};

template <typename _Scalar, int _N>
struct is_ad<ceres::Jet<_Scalar, _N>> : std::integral_constant<bool, true> { };

} /* namespace internal */

#ifdef _MANIF_MANIF_SO2_H_
using CeresConstraintSO2 = CeresConstraintFunctor<SO2d>;
#if CERES_VERSION_MAJOR >= 2 && CERES_VERSION_MINOR >= 2
using CeresManifoldSO2 = CeresManifoldFunctor<SO2d>;
#else
using CeresLocalParameterizationSO2 = CeresLocalParameterizationFunctor<SO2d>;
#endif
using CeresObjectiveSO2 = CeresObjectiveFunctor<SO2d>;
#else
using CeresConstraintSO2 = internal::YOU_MUST_INCLUDE_MANIF_BEFORE_CERES_HELPER_HEADERS;
using CeresManifoldSO2 = internal::YOU_MUST_INCLUDE_MANIF_BEFORE_CERES_HELPER_HEADERS;
using CeresObjectiveSO2 = internal::YOU_MUST_INCLUDE_MANIF_BEFORE_CERES_HELPER_HEADERS;
#endif

#ifdef _MANIF_MANIF_SO3_H_
using CeresConstraintSO3 = CeresConstraintFunctor<SO3d>;
#if CERES_VERSION_MAJOR >= 2 && CERES_VERSION_MINOR >= 2
using CeresManifoldSO3 = CeresManifoldFunctor<SO3d>;
#else
using CeresLocalParameterizationSO3 = CeresLocalParameterizationFunctor<SO3d>;
#endif
using CeresObjectiveSO3 = CeresObjectiveFunctor<SO3d>;
#else
using CeresConstraintSO3 = internal::YOU_MUST_INCLUDE_MANIF_BEFORE_CERES_HELPER_HEADERS;
using CeresManifoldSO3 = internal::YOU_MUST_INCLUDE_MANIF_BEFORE_CERES_HELPER_HEADERS;
using CeresObjectiveSO3 = internal::YOU_MUST_INCLUDE_MANIF_BEFORE_CERES_HELPER_HEADERS;
#endif

#ifdef _MANIF_MANIF_SE2_H_
using CeresConstraintSE2 = CeresConstraintFunctor<SE2d>;
#if CERES_VERSION_MAJOR >= 2 && CERES_VERSION_MINOR >= 2
using CeresManifoldSE2 = CeresManifoldFunctor<SE2d>;
#else
using CeresLocalParameterizationSE2 = CeresLocalParameterizationFunctor<SE2d>;
#endif
using CeresObjectiveSE2 = CeresObjectiveFunctor<SE2d>;
#else
using CeresConstraintSE2 = internal::YOU_MUST_INCLUDE_MANIF_BEFORE_CERES_HELPER_HEADERS;
using CeresManifoldSE2 = internal::YOU_MUST_INCLUDE_MANIF_BEFORE_CERES_HELPER_HEADERS;
using CeresObjectiveSE2 = internal::YOU_MUST_INCLUDE_MANIF_BEFORE_CERES_HELPER_HEADERS;
#endif

#ifdef _MANIF_MANIF_SE3_H_
using CeresConstraintSE3 = CeresConstraintFunctor<SE3d>;
#if CERES_VERSION_MAJOR >= 2 && CERES_VERSION_MINOR >= 2
using CeresManifoldSE3 = CeresManifoldFunctor<SE3d>;
#else
using CeresLocalParameterizationSE3 = CeresLocalParameterizationFunctor<SE3d>;
#endif
using CeresObjectiveSE3 = CeresObjectiveFunctor<SE3d>;
#else
using CeresConstraintSE3 = internal::YOU_MUST_INCLUDE_MANIF_BEFORE_CERES_HELPER_HEADERS;
using CeresManifoldSE3 = internal::YOU_MUST_INCLUDE_MANIF_BEFORE_CERES_HELPER_HEADERS;
using CeresObjectiveSE3= internal::YOU_MUST_INCLUDE_MANIF_BEFORE_CERES_HELPER_HEADERS;
#endif

} /* namespace manif */

#endif /* _MANIF_MANIF_CERES_CERES_H_ */
