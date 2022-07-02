#ifndef _MANIF_MANIF_IMPL_CONDITIONAL_OP_H_
#define _MANIF_MANIF_IMPL_CONDITIONAL_OP_H_

namespace manif {
namespace internal {

/// @brief Lower than conditional trait class
template <typename _Scalar>
struct CondOpLtHelper {
  static _Scalar eval(
    const _Scalar lhs, const _Scalar rhs, const _Scalar& vt, const _Scalar& vf
  ) {
    return (lhs < rhs) ? vt : vf;
  }

  template <typename ResultType>
  static ResultType eval(
    const _Scalar lhs, const _Scalar rhs, const ResultType& vt, const ResultType& vf
  ) {
    return (lhs < rhs) ? vt : vf;
  }
};

/// @brief Greater than conditional trait class
template <typename _Scalar>
struct CondOpGtHelper {
  static _Scalar eval(
    const _Scalar lhs, const _Scalar rhs, const _Scalar& vt, const _Scalar& vf
  ) {
    return (lhs > rhs) ? vt : vf;
  }

  template <typename ResultType>
  static ResultType eval(
    const _Scalar lhs, const _Scalar rhs, const ResultType& vt, const ResultType& vf
  ) {
    return (lhs > rhs) ? vt : vf;
  }
};

} // namespace internal

/**
 * @brief Lower than conditional helper function
 * @note (lhs < rhs)? true_value : false_value
 */
template <typename _Scalar, typename... Args>
auto if_lt(_Scalar&& a, Args&&... args)
-> decltype(
    internal::CondOpLtHelper<_Scalar>::eval(
      std::forward<_Scalar>(a), std::forward<Args>(args)...
    )
  )
{
  return internal::CondOpLtHelper<_Scalar>::eval(
    std::forward<_Scalar>(a), std::forward<Args>(args)...
  );
}

/**
 * @brief Greater than conditional helper function
 * @note (lhs > rhs)? true_value : false_value
 */
template <typename _Scalar, typename... Args>
auto if_gt(_Scalar&& a, Args&&... args)
-> decltype(
    internal::CondOpGtHelper<_Scalar>::eval(
      std::forward<_Scalar>(a), std::forward<Args>(args)...
    )
  )
{
  return internal::CondOpGtHelper<_Scalar>::eval(
    std::forward<_Scalar>(a), std::forward<Args>(args)...
  );
}

// @todo? if_le / if_ge / if_eq

} // namespace manif

#endif // _MANIF_MANIF_IMPL_CONDITIONAL_OP_H_