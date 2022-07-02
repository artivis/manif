#ifndef _MANIF_MANIF_CPPAD_CONDITIONAL_OP_H_
#define _MANIF_MANIF_CPPAD_CONDITIONAL_OP_H_

namespace manif {
namespace internal {

template <typename _Base>
struct CondOpLtHelper<CppAD::AD<_Base>> {
  using Scalar = CppAD::AD<_Base>;

  static Scalar eval(
    const Scalar& lhs, const Scalar& rhs, const Scalar& vt, const Scalar& vf
  ) {
    return CppAD::CondExpLt(lhs, rhs, vt, vf);
  }
};

template <typename _Base>
struct CondOpGtHelper<CppAD::AD<_Base>> {
  using Scalar = CppAD::AD<_Base>;

  static Scalar eval(
    const Scalar& lhs, const Scalar& rhs, const Scalar& vt, const Scalar& vf
  ) {
    return CppAD::CondExpGt(lhs, rhs, vt, vf);
  }

  template <typename Derived>
  static Eigen::Matrix<typename Derived::Scalar, Derived::Rows, 1> eval(
    const Scalar& lhs,
    const Scalar& rhs,
    const Eigen::MatrixBase<Derived>& vt,
    const Eigen::MatrixBase<Derived>& vf
  ) {
    using Scalar = typename Derived::Scalar;
    static constexpr unsigned int Size = Derived::Rows;
    Eigen::Matrix<Scalar, Size, 1> ret;

    for (int i=0; i<Size; ++i) {
      ret[i] = CppAD::CondExpGt(lhs, rhs, vt[i], vf[i]);
    }

    return ret;
  }
};

} // namespace internal
} // namespace manif

#endif // _MANIF_MANIF_CPPAD_CONDITIONAL_OP_H_