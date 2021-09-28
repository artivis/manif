#ifndef _MANIF_MANIF_CPPAD_CPPAD_H_
#define _MANIF_MANIF_CPPAD_CPPAD_H_

#include <cppad/cppad.hpp>

#include "manif/cppad/constants.h"
#include "manif/cppad/eigen.h"
#include "manif/cppad/local_parameterization.h"
#include "manif/cppad/conditional_op.h"

namespace manif {
namespace internal {
template <typename Scalar>
struct is_ad<CppAD::AD<Scalar>> : std::integral_constant<bool, true> { };
} // namespace internal
} // namespace manif

#endif // _MANIF_MANIF_CPPAD_CPPAD_H_