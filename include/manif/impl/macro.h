#ifndef _MANIF_MANIF_FWD_H_
#define _MANIF_MANIF_FWD_H_

#include <stdexcept> // for std::runtime_error

#ifdef NDEBUG
# ifndef MANIF_NO_DEBUG
#  define MANIF_NO_DEBUG
# endif
#endif

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

// Assertions cost run time and can be turned off.
// You can suppress MANIF_ASSERT by defining
// MANIF_NO_DEBUG before including manif headers.
// MANIF_NO_DEBUG is undefined by default unless NDEBUG is defined.
#ifndef MANIF_NO_DEBUG
  #define MANIF_ASSERT(...)                         \
    __MANIF_EXPAND(                                 \
    __MANIF_GET_MACRO_3(__VA_ARGS__,                \
                        __MANIF_CHECK_MSG_EXCEPT,   \
                        __MANIF_CHECK_MSG,          \
                        __MANIF_CHECK)(__VA_ARGS__) )
#else
  #define MANIF_ASSERT(...) ((void)0)
#endif

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

// Common macros

#define MANIF_MAKE_ALIGNED_OPERATOR_NEW_COND \
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW_IF((Eigen::internal::traits<typename Base::DataType>::Alignment>0))
#define MANIF_MAKE_ALIGNED_OPERATOR_NEW_COND_TYPE(X) \
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW_IF((Eigen::internal::traits<typename X::DataType>::Alignment>0))

#define MANIF_MOVE_NOEXCEPT \
  noexcept(std::is_nothrow_move_constructible<Scalar>::value)

#define MANIF_DEFAULT_CONSTRUCTOR(X)  \
  X() = default;                      \
  ~X() = default;                     \
  X(const X&) = default;              \
  X(X&&) = default;

#define MANIF_GROUP_ML_ASSIGN_OP(X) \
  _Derived& operator =(const X& o) { coeffs() = o.coeffs(); return derived(); }\
  template <typename _DerivedOther>\
  _Derived& operator =(const LieGroupBase<_DerivedOther>& o) { coeffs() = o.coeffs(); return derived(); }\
  template <typename _EigenDerived>\
  _Derived& operator =(const Eigen::MatrixBase<_EigenDerived>& o) { coeffs() = o; return derived(); } \
  _Derived& operator =(X&& o) MANIF_MOVE_NOEXCEPT { coeffs() = std::move(o.coeffs()); return derived(); }\
  template <typename _DerivedOther>\
  _Derived& operator =(LieGroupBase<_DerivedOther>&& o) MANIF_MOVE_NOEXCEPT { coeffs() = std::move(o.coeffs()); return derived(); }\
  template <typename _EigenDerived>\
  _Derived& operator =(Eigen::MatrixBase<_EigenDerived>&& o) { coeffs() = std::move(o); return derived(); }

#define MANIF_GROUP_ASSIGN_OP(X) \
  X& operator=(const X& o) { coeffs() = o.coeffs(); return derived(); }\
  template <typename _DerivedOther>\
  X& operator =(const X##Base<_DerivedOther>& o) { coeffs() = o.coeffs(); return derived(); }\
  template <typename _DerivedOther>\
  X& operator =(const LieGroupBase<_DerivedOther>& o) { coeffs() = o.coeffs(); return derived(); }\
  template <typename _EigenDerived>\
  X& operator =(const Eigen::MatrixBase<_EigenDerived>& o) { coeffs() = o; return derived(); }\
  X& operator=(X&& o) MANIF_MOVE_NOEXCEPT { coeffs() = std::move(o.coeffs()); return derived(); }\
  template <typename _DerivedOther>\
  X& operator =(X##Base<_DerivedOther>&& o) MANIF_MOVE_NOEXCEPT { coeffs() = std::move(o.coeffs()); return derived(); }\
  template <typename _DerivedOther>\
  X& operator =(LieGroupBase<_DerivedOther>&& o) MANIF_MOVE_NOEXCEPT { coeffs() = std::move(o.coeffs()); return derived(); }\
  template <typename _EigenDerived>\
  X& operator =(Eigen::MatrixBase<_EigenDerived>&& o) { coeffs() = std::move(o); return derived(); }

#define MANIF_GROUP_MAP_ASSIGN_OP(X) \
  Map& operator=(const Map& o) { coeffs() = o.coeffs(); return *this; }\
  template <typename _DerivedOther>\
  Map& operator =(const manif::X##Base<_DerivedOther>& o) { coeffs() = o.coeffs(); return *this; }\
  template <typename _DerivedOther>\
  Map& operator =(const manif::LieGroupBase<_DerivedOther>& o) { coeffs() = o.coeffs(); return *this; }\
  template <typename _EigenDerived>\
  Map& operator =(const Eigen::MatrixBase<_EigenDerived>& o) { coeffs() = o; return *this; }\
  Map& operator=(Map&& o) MANIF_MOVE_NOEXCEPT { coeffs() = std::move(o.coeffs()); return *this; }\
  template <typename _DerivedOther>\
  Map& operator =(manif::X##Base<_DerivedOther>&& o) MANIF_MOVE_NOEXCEPT { coeffs() = std::move(o.coeffs()); return *this; }\
  template <typename _DerivedOther>\
  Map& operator =(manif::LieGroupBase<_DerivedOther>&& o) MANIF_MOVE_NOEXCEPT { coeffs() = std::move(o.coeffs()); return *this; }\
  template <typename _EigenDerived>\
  Map& operator =(Eigen::MatrixBase<_EigenDerived>&& o) { coeffs() = std::move(o); return *this; }

#define MANIF_TANGENT_ML_ASSIGN_OP(X) \
  _Derived& operator=(const X& o) { coeffs() = o.coeffs(); return derived(); }\
  template <typename _DerivedOther>\
  _Derived& operator =(const TangentBase<_DerivedOther>& o) { coeffs() = o.coeffs(); return derived(); }\
  template <typename _EigenDerived>\
  _Derived& operator =(const Eigen::MatrixBase<_EigenDerived>& o) { coeffs() = o; return derived(); }\
  _Derived& operator=(X&& o) MANIF_MOVE_NOEXCEPT { coeffs() = std::move(o.coeffs()); return derived(); }\
  template <typename _DerivedOther>\
  _Derived& operator =(TangentBase<_DerivedOther>&& o) MANIF_MOVE_NOEXCEPT { coeffs() = std::move(o.coeffs()); return derived(); }\
  template <typename _EigenDerived>\
  _Derived& operator =(Eigen::MatrixBase<_EigenDerived>&& o) MANIF_MOVE_NOEXCEPT { coeffs() = std::move(o); return derived(); }

#define MANIF_TANGENT_ASSIGN_OP(X) \
  X& operator=(const X& o) { coeffs() = o.coeffs(); return derived(); }\
  template <typename _DerivedOther>\
  X& operator =(const X##Base<_DerivedOther>& o) { coeffs() = o.coeffs(); return derived(); }\
  template <typename _DerivedOther>\
  X& operator =(const TangentBase<_DerivedOther>& o) { coeffs() = o.coeffs(); return derived(); }\
  template <typename _EigenDerived>\
  X& operator =(const Eigen::MatrixBase<_EigenDerived>& o) { coeffs() = o; return derived(); }\
  X& operator=(X&& o) MANIF_MOVE_NOEXCEPT { coeffs() = std::move(o.coeffs()); return derived(); }\
  template <typename _DerivedOther>\
  X& operator =(X##Base<_DerivedOther>&& o) MANIF_MOVE_NOEXCEPT { coeffs() = std::move(o.coeffs()); return derived(); }\
  template <typename _DerivedOther>\
  X& operator =(TangentBase<_DerivedOther>&& o) MANIF_MOVE_NOEXCEPT { coeffs() = std::move(o.coeffs()); return derived(); }\
  template <typename _EigenDerived>\
  X& operator =(Eigen::MatrixBase<_EigenDerived>&& o) MANIF_MOVE_NOEXCEPT { coeffs() = std::move(o); return derived(); }

#define MANIF_TANGENT_MAP_ASSIGN_OP(X) \
  Map& operator=(const Map& o) { coeffs() = o.coeffs(); return *this; }\
  template <typename _DerivedOther>\
  Map& operator =(const manif::X##Base<_DerivedOther>& o) { coeffs() = o.coeffs(); return *this; }\
  template <typename _DerivedOther>\
  Map& operator =(const manif::TangentBase<_DerivedOther>& o) { coeffs() = o.coeffs(); return *this; }\
  template <typename _EigenDerived>\
  Map& operator =(const Eigen::MatrixBase<_EigenDerived>& o) { coeffs() = o; return *this; }\
  Map& operator=(Map&& o) MANIF_MOVE_NOEXCEPT { coeffs() = std::move(o.coeffs()); return *this; }\
  template <typename _DerivedOther>\
  Map& operator =(manif::X##Base<_DerivedOther>&& o) MANIF_MOVE_NOEXCEPT { coeffs() = std::move(o.coeffs()); return *this; }\
  template <typename _DerivedOther>\
  Map& operator =(manif::TangentBase<_DerivedOther>&& o) MANIF_MOVE_NOEXCEPT { coeffs() = std::move(o.coeffs()); return *this; }\
  template <typename _EigenDerived>\
  Map& operator =(Eigen::MatrixBase<_EigenDerived>&& o) MANIF_MOVE_NOEXCEPT { coeffs() = std::move(o); return *this; }

/**
 * @brief Automatically define:
 * - copy constructor
 * - copy constructor given Base object
 * - copy constructor given Eigen object
 */
#define MANIF_COPY_CONSTRUCTOR(X)                                           \
  X(const X& o) : Base(), data_(o.coeffs()) { }                             \
  X(const Base& o) : Base(), data_(o.coeffs()) { }                          \
  template <typename D> X(const Eigen::MatrixBase<D>& o) : Base(), data_(o) \
    { manif::internal::AssignmentEvaluator<Base>().run(data_); }

#define MANIF_MOVE_CONSTRUCTOR(X)                                                 \
  X(X&& o) MANIF_MOVE_NOEXCEPT : Base(), data_(std::move(o.coeffs())) { }         \
  X(Base&& o) MANIF_MOVE_NOEXCEPT : Base(), data_(std::move(o.coeffs())) { }      \
  template <typename D> X(Eigen::MatrixBase<D>&& o) : Base(), data_(std::move(o)) \
    { manif::internal::AssignmentEvaluator<Base>().run(data_); }

#define MANIF_COEFFS_FUNCTIONS()                      \
  DataType& coeffs() & { return data_; }              \
  const DataType& coeffs() const & { return data_; }

// LieGroup - related macros

#define MANIF_INHERIT_GROUP_AUTO_API    \
  using Base::setRandom;                \
  using Base::rplus;                    \
  using Base::lplus;                    \
  using Base::rminus;                   \
  using Base::lminus;                   \
  using Base::between;

#define MANIF_INHERIT_GROUP_API     \
  MANIF_INHERIT_GROUP_AUTO_API      \
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

#define MANIF_GROUP_PROPERTIES \
  using Base::Dim;             \
  using Base::DoF;

#define MANIF_GROUP_TYPEDEF                             \
  MANIF_GROUP_PROPERTIES                                \
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

#define MANIF_TANGENT_PROPERTIES \
using Base::Dim;                 \
using Base::DoF;

#define MANIF_TANGENT_TYPEDEF               \
  MANIF_TANGENT_PROPERTIES                  \
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
