#ifndef _MANIF_MANIF_CPPAD_CONSTANTS_H_
#define _MANIF_MANIF_CPPAD_CONSTANTS_H_

namespace manif {

/// @brief Specialize Constants traits for the float-based CppAD::AD type
template <>
struct Constants<CppAD::AD<float>> {
  static const CppAD::AD<float> eps;
};

const CppAD::AD<float>
Constants<CppAD::AD<float>>::eps = CppAD::AD<float>(1e-6);

/// @brief Specialize Constants traits for the double-based CppAD::AD type
template <>
struct Constants<CppAD::AD<double>> {
  static const CppAD::AD<double> eps;
};

const CppAD::AD<double>
Constants<CppAD::AD<double>>::eps = CppAD::AD<double>(1e-14);

} // namespace manif

#endif // _MANIF_MANIF_CPPAD_CONSTANTS_H_