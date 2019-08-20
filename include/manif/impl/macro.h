#ifndef _MANIF_MANIF_FWD_H_
#define _MANIF_MANIF_FWD_H_

#include <stdexcept> // for std::runtime_error

namespace manif {

struct runtime_error : std::runtime_error
{
  using std::runtime_error::runtime_error;
  using std::runtime_error::what;
};

struct invalid_argument : std::invalid_argument
{
  using std::invalid_argument::invalid_argument;
  using std::invalid_argument::what;
};

namespace detail {

template <typename E, typename... Args>
void
#if defined(__GNUC__) || defined(__clang__)
__attribute__(( noinline, cold, noreturn ))
#elif defined(_MSC_VER)
__declspec( noinline, noreturn )
#else
// nothing
#endif
raise(Args&&... args)
{
  throw E(std::forward<Args>(args)...);
}
} /* namespace detail */
} /* namespace manif */

// gcc expands __VA_ARGS___ before passing it into the macro.
// Visual Studio expands __VA_ARGS__ after passing it.
// This macro is a workaround to support both
#define __MANIF_EXPAND(x) x

#if defined(__cplusplus) && defined(__has_cpp_attribute)
  #define __MANIF_HAVE_CPP_ATTRIBUTE(x) __has_cpp_attribute(x)
#else
  #define __MANIF_HAVE_CPP_ATTRIBUTE(x) 0
#endif

#define __MANIF_THROW_EXCEPT(msg, except) manif::detail::raise<except>(msg);
#define __MANIF_THROW(msg) __MANIF_THROW_EXCEPT(msg, manif::runtime_error)

#define __MANIF_GET_MACRO_2(_1,_2,NAME,...) NAME

#define MANIF_THROW(...)                          \
  __MANIF_EXPAND(                                 \
  __MANIF_GET_MACRO_2(__VA_ARGS__,                \
                      __MANIF_THROW_EXCEPT,       \
                      __MANIF_THROW)(__VA_ARGS__) )

#define __MANIF_CHECK_MSG_EXCEPT(cond, msg, except) \
  if (!(cond)) {MANIF_THROW(msg, except);}
#define __MANIF_CHECK_MSG(cond, msg) \
  __MANIF_CHECK_MSG_EXCEPT(cond, msg, manif::runtime_error)
#define __MANIF_CHECK(cond) \
  __MANIF_CHECK_MSG_EXCEPT(cond, "Condition: '"#cond"' failed!", manif::runtime_error)

#define __MANIF_GET_MACRO_3(_1,_2,_3,NAME,...) NAME

#define MANIF_CHECK(...)                          \
  __MANIF_EXPAND(                                 \
  __MANIF_GET_MACRO_3(__VA_ARGS__,                \
                      __MANIF_CHECK_MSG_EXCEPT,   \
                      __MANIF_CHECK_MSG,          \
                      __MANIF_CHECK)(__VA_ARGS__) )

#define MANIF_NOT_IMPLEMENTED_YET \
  MANIF_THROW("Not implemented yet !");

#if defined(__cplusplus)  && (__cplusplus >= 201402L) && __MANIF_HAVE_CPP_ATTRIBUTE(deprecated)
  #define MANIF_DEPRECATED [[deprecated]]
#elif defined(__GNUC__)  || defined(__clang__)
  #define MANIF_DEPRECATED __attribute__((deprecated))
#elif defined(_MSC_VER)
  #define MANIF_DEPRECATED __declspec(deprecated)
#else
  #pragma message("WARNING: Deprecation is disabled "\
                  "-- the compiler is not supported.")
  #define MANIF_DEPRECATED
#endif

// LieGroup - related macros

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
  using Base::log;                  \
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

// Tangent - related macros

#define MANIF_TANGENT_PROPERTIES                                      \
  static constexpr int Dim = internal::LieGroupProperties<Type>::Dim; \
  static constexpr int DoF = internal::LieGroupProperties<Type>::DoF;

#define MANIF_INHERIT_TANGENT_API \
  using Base::setZero;            \
  using Base::setRandom;          \
  using Base::retract;            \
  using Base::exp;                \
  using Base::hat;                \
  using Base::rjac;               \
  using Base::ljac;               \
  using Base::smallAdj;

#define MANIF_INHERIT_TANGENT_OPERATOR \
  using Base::operator +=;             \
  using Base::operator -=;             \
  using Base::operator *=;             \
  using Base::operator /=;             \
  using Base::operator =;              \
  using Base::operator <<;

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
