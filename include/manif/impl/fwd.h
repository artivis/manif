#ifndef _MANIF_MANIF_FWD_H_
#define _MANIF_MANIF_FWD_H_

#include <type_traits>

#include <stdexcept> // for std::runtime_error

namespace manif
{

//using WithJacobian = std::true_type;
//using WithoutJacobian = std::false_type;

namespace internal
{

template <typename T> struct traits;

/// @note the following is from the Eigen library

// here we say once and for all that traits<const T> == traits<T>
// When constness must affect traits, it has to be constness on
// template parameters on which T itself depends.
// For example, traits<Map<const T> > != traits<Map<T> >, but
//              traits<const Map<T> > == traits<Map<T> >
template<typename T> struct traits<const T> : traits<T> {};

/// @brief ManifoldProperties
/**
 * @brief ManifoldProperties, a traits for defining some
 * manifold properties. E.g. Space dimension, Degrees of Freedom ...
 */
template <typename _ManifoldBase> struct ManifoldProperties;

} /* namespace internal */
} /* namespace manif */

#define MANIF_NOT_IMPLEMENTED_YET \
  throw std::runtime_error("Not implemented yet !");

#define MANIF_INHERIT_MANIFOLD_API  \
  using Base::matrix;               \
  using Base::rotation;             \
  using Base::identity;             \
  using Base::random;               \
  using Base::inverse;              \
  using Base::rplus;                \
  using Base::lplus;                \
  using Base::rminus;               \
  using Base::lminus;               \
  using Base::lift;                 \
  using Base::compose;              \
  using Base::between;

#define MANIF_INHERIT_TANGENT_API \
  using Base::zero;               \
  using Base::random;             \
  using Base::retract;

#define MANIF_DEFINE_MANIFOLD_TANGENT                    \
  struct Tangent : TangentBase<typename Base::Manifold>  \
  {                                                      \
    void zero();                                         \
    void random();                                       \
    typename Base::Manifold retract() const;             \
  };

#define MANIF_MANIFOLD_BASE_TYPEDEF           \
  using Base::operator +;                     \
  using Base::operator +=;                    \
  using Base::operator *;                     \
  using Base::operator *=;                    \
  using Manifold = typename Base::Manifold;

#define MANIF_MANIFOLD_TYPEDEF                              \
  using Scalar = _Scalar;                                   \
  using Manifold = typename Base::Manifold;                 \
  using Tangent = typename Base::Tangent;                   \
  using Base::Dim;                                          \
  using ManifoldDataType = typename Base::ManifoldDataType; \
  using Jacobian = typename Base::Jacobian;                 \
  using Base::RepSize;

#define MANIF_INHERIT_MANIFOLD_OPERATOR \
  using Base::operator +;               \
  using Base::operator +=;              \
  using Base::operator *;               \
  using Base::operator *=;              \
  using Base::operator =;

#define MANIF_COMPLETE_MANIFOLD_TYPEDEF \
  MANIF_MANIFOLD_TYPEDEF                \
  MANIF_INHERIT_MANIFOLD_OPERATOR

#define MANIF_EXTRA_MANIFOLD_TYPEDEF(manifold) \
  using manifold##f = manifold<float>;         \
  using manifold##d = manifold<double>;

#define MANIF_EXTRA_TANGENT_TYPEDEF(tangent) \
  using tangent##f = tangent<float>;         \
  using tangent##d = tangent<double>;

#endif /* _MANIF_MANIF_FWD_H_ */
