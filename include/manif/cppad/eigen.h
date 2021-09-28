#ifndef _MANIF_MANIF_CPPAD_EIGEN_H_
#define _MANIF_MANIF_CPPAD_EIGEN_H_

namespace Eigen {
namespace internal {

/// @note Eigen cast specialization
/// enables Eigen::MatrixXd = MatrixXad.cast<double>();
template <typename T>
struct cast_impl<CppAD::AD<T>, T> {
  EIGEN_DEVICE_FUNC static inline T run(const CppAD::AD<T>& x) {
    return CppAD::Value(x);
  }
};

/// @note Eigen cast specialization to prevents nesting AD
/// e.g. CppAD::AD<CppAD::AD<T>>
template <typename T>
struct cast_impl<CppAD::AD<T>, CppAD::AD<T>> {
  EIGEN_DEVICE_FUNC static inline T run(const CppAD::AD<T>& x) {
    return x;
  }
};

} // namespace internal
} // namespace Eigen

#endif // _MANIF_MANIF_CPPAD_EIGEN_H_