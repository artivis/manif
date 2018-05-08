#ifndef _MANIF_MANIF_FWD_H_
#define _MANIF_MANIF_FWD_H_

#include <stdexcept> // for std::runtime_error

#define MANIF_NOT_IMPLEMENTED_YET \
  throw std::runtime_error("Not implemented yet !");

/// Manifold - related macros

#define MANIF_MANIFOLD_PROPERTIES                                     \
  static constexpr int Dim = internal::ManifoldProperties<Type>::Dim; \
  static constexpr int DoF = internal::ManifoldProperties<Type>::DoF; \
  static constexpr int N   = internal::ManifoldProperties<Type>::N;

#define MANIF_INHERIT_MANIFOLD_AUTO_API \
  using Base::random;                   \
  using Base::rplus;                    \
  using Base::lplus;                    \
  using Base::rminus;                   \
  using Base::lminus;                   \
  using Base::between;

#define MANIF_INHERIT_MANIFOLD_API  \
  MANIF_INHERIT_MANIFOLD_AUTO_API   \
  using Base::transform;            \
  using Base::rotation;             \
  using Base::identity;             \
  using Base::inverse;              \
  using Base::lift;

#define MANIF_MANIFOLD_TYPEDEF                          \
  using Scalar         = typename Base::Scalar;         \
  using Manifold       = typename Base::Manifold;       \
  using Tangent        = typename Base::Tangent;        \
  using Jacobian       = typename Base::Jacobian;       \
  using DataType       = typename Base::DataType;       \
  using Transformation = typename Base::Transformation; \
  using Rotation       = typename Base::Rotation;       \
  using Vector         = typename Base::Vector;         \
  using OptJacobianRef = typename Base::OptJacobianRef;

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

/// Tangent - related macros

#define MANIF_INHERIT_TANGENT_API \
  using Base::zero;               \
  using Base::random;             \
  using Base::retract;            \
  using Base::skew;

#define MANIF_INHERIT_TANGENT_OPERATOR \
  using Base::operator +;              \
  using Base::operator =;

#define MANIF_TANGENT_TYPEDEF               \
  using Scalar   = typename Base::Scalar;   \
  using Manifold = typename Base::Manifold; \
  using Tangent  = typename Base::Tangent;  \
  using Jacobian = typename Base::Jacobian; \
  using DataType = typename Base::DataType; \
  using LieType  = typename Base::LieType;  \
  using OptJacobianRef = typename Base::OptJacobianRef;

#define MANIF_TANGENT_PROPERTIES                                      \
  static constexpr int Dim = internal::ManifoldProperties<Type>::Dim; \
  static constexpr int DoF = internal::ManifoldProperties<Type>::DoF;

#define MANIF_EXTRA_TANGENT_TYPEDEF(tangent) \
  using tangent##f = tangent<float>;         \
  using tangent##d = tangent<double>;

#endif /* _MANIF_MANIF_FWD_H_ */
