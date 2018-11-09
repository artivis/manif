#ifndef _MANIF_MANIF_CPPAD_CONSTANTS_H_
#define _MANIF_MANIF_CPPAD_CONSTANTS_H_

#include <cppad/base_require.hpp>

namespace manif {

template <typename _Scalar>
struct Constants<CppAD::AD<_Scalar>>
{
  static const CppAD::AD<_Scalar> eps;
  static const CppAD::AD<_Scalar> eps_s;
  static const CppAD::AD<_Scalar> eps_sqrt;

  static const CppAD::AD<_Scalar> to_rad;
  static const CppAD::AD<_Scalar> to_deg;
};

template <typename _Scalar>
const CppAD::AD<_Scalar>
Constants<CppAD::AD<_Scalar>>::eps      = CppAD::AD<_Scalar>(1e-10);

template <typename _Scalar>
const CppAD::AD<_Scalar>
Constants<CppAD::AD<_Scalar>>::eps_s    = CppAD::AD<_Scalar>(1e-15);

template <typename _Scalar>
const CppAD::AD<_Scalar>
Constants<CppAD::AD<_Scalar>>::eps_sqrt = CppAD::sqrt(eps);

template <typename _Scalar>
const CppAD::AD<_Scalar>
Constants<CppAD::AD<_Scalar>>::to_rad   = CppAD::AD<_Scalar>(M_PI / 180);

template <typename _Scalar>
const CppAD::AD<_Scalar>
Constants<CppAD::AD<_Scalar>>::to_deg   = CppAD::AD<_Scalar>(180.0 / M_PI);

} /* namespace manif  */

#endif /* _MANIF_MANIF_CPPAD_CONSTANTS_H_ */
