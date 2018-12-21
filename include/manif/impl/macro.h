#ifndef _MANIF_MANIF_FWD_H_
#define _MANIF_MANIF_FWD_H_

#include <stdexcept> // for std::runtime_error

namespace manif {
namespace detail {
template <typename E, typename... Args>
#ifdef _MANIF_COMPILER_SUPPORTS_CONSTEXPR_VOID_
constexpr void
#else
void
#endif
__attribute__(( noinline, cold, noreturn )) raise(Args&&... args)
{
  throw E(std::forward<Args>(args)...);
}
} /* namespace detail */
} /* namespace manif */

#define MANIF_THROW(msg) \
  manif::detail::raise<std::runtime_error>(msg);

#define MANIF_NOT_IMPLEMENTED_YET \
  MANIF_THROW("Not implemented yet !");

#define MANIF_CHECK(cond, msg) \
  if (!(cond)) MANIF_THROW(msg);

/// LieGroup - related macros

#define MANIF_GROUP_PROPERTIES                                        \
  static constexpr int Dim = internal::LieGroupProperties<Type>::Dim; \
  static constexpr int DoF = internal::LieGroupProperties<Type>::DoF;

#define MANIF_INHERIT_GROUP_AUTO_API    \
  using Base::setRandom;                \
  using Base::rplus;                    \
  using Base::lplus;                    \
  using Base::rminus;                   \
  using Base::lminus;                   \
  using Base::between;

#define MANIF_INHERIT_GROUP_API     \
  MANIF_INHERIT_GROUP_AUTO_API      \
  using Base::transform;            \
  using Base::rotation;             \
  using Base::setIdentity;          \
  using Base::inverse;              \
  using Base::lift;                 \
  using Base::adj;

#define MANIF_INHERIT_GROUP_OPERATOR    \
  using Base::operator +;               \
  using Base::operator +=;              \
  using Base::operator -;               \
  using Base::operator *;               \
  using Base::operator *=;              \
  using Base::operator =;

#define MANIF_GROUP_TYPEDEF                             \
  using Scalar         = typename Base::Scalar;         \
  using LieGroup       = typename Base::LieGroup;       \
  using Tangent        = typename Base::Tangent;        \
  using Jacobian       = typename Base::Jacobian;       \
  using DataType       = typename Base::DataType;       \
  using Vector         = typename Base::Vector;         \
  using OptJacobianRef = typename Base::OptJacobianRef;

#define MANIF_COMPLETE_GROUP_TYPEDEF \
  MANIF_GROUP_TYPEDEF                \
  MANIF_INHERIT_GROUP_OPERATOR

#define MANIF_EXTRA_GROUP_TYPEDEF(group) \
  using group##f = group<float>;         \
  using group##d = group<double>;

/// Tangent - related macros

#define MANIF_TANGENT_PROPERTIES                                      \
  static constexpr int Dim = internal::LieGroupProperties<Type>::Dim; \
  static constexpr int DoF = internal::LieGroupProperties<Type>::DoF;

#define MANIF_INHERIT_TANGENT_API \
  using Base::setZero;            \
  using Base::setRandom;          \
  using Base::retract;            \
  using Base::hat;                \
  using Base::rjac;               \
  using Base::ljac;               \
  using Base::smallAdj;

#define MANIF_INHERIT_TANGENT_OPERATOR \
  using Base::operator +=;             \
  using Base::operator -=;             \
  using Base::operator *=;             \
  using Base::operator /=;             \
  using Base::operator =;

#define MANIF_TANGENT_TYPEDEF               \
  using Scalar   = typename Base::Scalar;   \
  using LieGroup = typename Base::LieGroup; \
  using Tangent  = typename Base::Tangent;  \
  using Jacobian = typename Base::Jacobian; \
  using DataType = typename Base::DataType; \
  using LieAlg   = typename Base::LieAlg;   \
  using OptJacobianRef = typename Base::OptJacobianRef;

#define MANIF_EXTRA_TANGENT_TYPEDEF(tangent) \
  using tangent##f = tangent<float>;         \
  using tangent##d = tangent<double>;

#endif /* _MANIF_MANIF_FWD_H_ */
