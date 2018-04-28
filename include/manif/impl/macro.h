#ifndef _MANIF_MANIF_FWD_H_
#define _MANIF_MANIF_FWD_H_

#include <stdexcept> // for std::runtime_error

#define MANIF_NOT_IMPLEMENTED_YET \
  throw std::runtime_error("Not implemented yet !");

#define MANIF_INHERIT_MANIFOLD_API  \
  using Base::transform;            \
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

#define MANIF_INHERIT_TANGENT_OPERATOR \
  using Base::operator +;              \
  using Base::operator =;

#define MANIF_TANGENT_TYPEDEF               \
  using Scalar   = typename Base::Scalar;   \
  using Manifold = typename Base::Manifold; \
  using Tangent  = typename Base::Tangent;  \
  using Jacobian = typename Base::Jacobian; \
  using DataType = typename Base::DataType;

#define MANIF_MANIFOLD_TYPEDEF              \
  using Scalar = _Scalar;                   \
  using Manifold = typename Base::Manifold; \
  using Tangent  = typename Base::Tangent;  \
  using Base::Dim;                          \
  using DataType = typename Base::DataType; \
  using Jacobian = typename Base::Jacobian; \
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
