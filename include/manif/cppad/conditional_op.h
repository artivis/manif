#ifndef _MANIF_MANIF_CPPAD_CONDITIONAL_OP_H_
#define _MANIF_MANIF_CPPAD_CONDITIONAL_OP_H_

#include <cppad/local/ad.hpp>

namespace manif {
namespace internal {

template <typename _Base>
struct CondOpLtHelper<CppAD::AD<_Base>>
{
private:

  using Scalar = CppAD::AD<_Base>;

public:

  static Scalar
  eval(const Scalar& lhs, const Scalar& rhs,
       const Scalar& vt, const Scalar& vf)
  {
    return CppAD::CondExpLt(lhs, rhs, vt, vf);
  }
};

template <typename _Base>
struct CondOpLeHelper<CppAD::AD<_Base>>
{
private:

  using Scalar = CppAD::AD<_Base>;

public:

  static Scalar
  eval(const Scalar& lhs, const Scalar& rhs,
       const Scalar& vt, const Scalar& vf)
  {
    return CppAD::CondExpLe(lhs, rhs, vt, vf);
  }
};

template <typename _Base>
struct CondOpEqHelper<CppAD::AD<_Base>>
{
private:

  using Scalar = CppAD::AD<_Base>;

public:

  static Scalar
  eval(const Scalar& lhs, const Scalar& rhs,
       const Scalar& vt, const Scalar& vf)
  {
    return CppAD::CondExpEq(lhs, rhs, vt, vf);
  }
};

template <typename _Base>
struct CondOpGeHelper<CppAD::AD<_Base>>
{
private:

  using Scalar = CppAD::AD<_Base>;

public:

  static Scalar
  eval(const Scalar& lhs, const Scalar& rhs,
       const Scalar& vt, const Scalar& vf)
  {
    return CppAD::CondExpGe(lhs, rhs, vt, vf);
  }
};

template <typename _Base>
struct CondOpGtHelper<CppAD::AD<_Base>>
{
private:

  using Scalar = CppAD::AD<_Base>;

public:

  static Scalar
  eval(const Scalar& lhs, const Scalar& rhs,
       const Scalar& vt, const Scalar& vf)
  {
    return CppAD::CondExpGt(lhs, rhs, vt, vf);
  }
};

} /* namespace internal */
} /* namespace manif */

#endif /* _MANIF_MANIF_CPPAD_CONDITIONAL_OP_H_ */
