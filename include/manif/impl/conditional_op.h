#ifndef _MANIF_MANIF_IMPL_CONDITIONAL_OP_H_
#define _MANIF_MANIF_IMPL_CONDITIONAL_OP_H_

namespace manif {
namespace internal {

template <typename _Scalar>
struct CondOpLtHelper
{
  static _Scalar eval(const _Scalar& lhs, const _Scalar& rhs,
                      const _Scalar& vt,  const _Scalar& vf)
  {
    return (lhs < rhs) ? vt : vf;
  }
};

template <typename _Scalar>
struct CondOpLeHelper
{
  static _Scalar eval(const _Scalar& lhs, const _Scalar& rhs,
                      const _Scalar& vt,  const _Scalar& vf)
  {
    return (lhs <= rhs) ? vt : vf;
  }
};

template <typename _Scalar>
struct CondOpEqHelper
{
  static _Scalar eval(const _Scalar& lhs, const _Scalar& rhs,
                      const _Scalar& vt,  const _Scalar& vf)
  {
    return (lhs == rhs) ? vt : vf;
  }
};

template <typename _Scalar>
struct CondOpGeHelper
{
  static _Scalar eval(const _Scalar& lhs, const _Scalar& rhs,
                      const _Scalar& vt,  const _Scalar& vf)
  {
    return (lhs >= rhs) ? vt : vf;
  }
};

template <typename _Scalar>
struct CondOpGtHelper
{
  static _Scalar eval(const _Scalar& lhs, const _Scalar& rhs,
                      const _Scalar& vt,  const _Scalar& vf)
  {
    return (lhs > rhs) ? vt : vf;
  }
};

} /* namespace internal */

template <typename _Scalar, typename... Args>
_Scalar if_lt(_Scalar&& a, Args&&... args)
{
  return internal::CondOpLtHelper<_Scalar>::eval(
        std::forward<_Scalar>(a), std::forward<Args>(args)...);
}

template <typename _Scalar, typename... Args>
_Scalar if_le(_Scalar&& a, Args&&... args)
{
  return internal::CondOpLeHelper<_Scalar>::eval(
        std::forward<_Scalar>(a), std::forward<Args>(args)...);
}

template <typename _Scalar, typename... Args>
_Scalar if_gt(_Scalar&& a, Args&&... args)
{
  return internal::CondOpGtHelper<_Scalar>::eval(
        std::forward<_Scalar>(a), std::forward<Args>(args)...);
}

template <typename _Scalar, typename... Args>
_Scalar if_ge(_Scalar&& a, Args&&... args)
{
  return internal::CondOpGeHelper<_Scalar>::eval(
        std::forward<_Scalar>(a), std::forward<Args>(args)...);
}

template <typename _Scalar, typename... Args>
_Scalar if_eq(_Scalar&& a, Args&&... args)
{
  return internal::CondOpEqHelper<_Scalar>::eval(
        std::forward<_Scalar>(a), std::forward<Args>(args)...);
}

/// @note warning: variable templates only available with -std=c++14
//template <typename _Scalar>
//const auto IfLt2 = &internal::CondOpLtHelper<_Scalar>::eval;

} /* namespace manif */

#endif /* _MANIF_MANIF_IMPL_CONDITIONAL_OP_H_ */
