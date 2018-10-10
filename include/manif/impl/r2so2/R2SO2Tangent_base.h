#ifndef _MANIF_MANIF_R2SO2TANGENT_BASE_H_
#define _MANIF_MANIF_R2SO2TANGENT_BASE_H_

#include "manif/impl/R2SO2/R2SO2_properties.h"
#include "manif/impl/tangent_base.h"

namespace manif
{

///////////////
///         ///
/// Tangent ///
///         ///
///////////////

template <typename _Derived>
struct R2SO2TangentBase : TangentBase<_Derived>
{
private:

  using Base = TangentBase<_Derived>;
  using Type = R2SO2TangentBase<_Derived>;

public:

  MANIF_TANGENT_PROPERTIES
  MANIF_TANGENT_TYPEDEF
  MANIF_INHERIT_TANGENT_OPERATOR

  using Base::data;
  using Base::coeffs;

  /// Tangent common API

  LieAlg hat() const;

  Manifold retract(OptJacobianRef J_m_t = {}) const;

  Jacobian rjac() const;
  Jacobian ljac() const;

  Jacobian adj() const;

  /// R2SO2Tangent specific API

  Scalar x() const;
  Scalar y() const;
  Scalar angle() const;
};

template <typename _Derived>
typename R2SO2TangentBase<_Derived>::LieAlg
R2SO2TangentBase<_Derived>::hat() const
{
  return ( LieAlg() <<
             Scalar(0), -angle(),   x(),
             angle(),    Scalar(0), y(),
             Scalar(0),  Scalar(0), Scalar(0) ).finished();
}

template <typename _Derived>
typename R2SO2TangentBase<_Derived>::Manifold
R2SO2TangentBase<_Derived>::retract(OptJacobianRef J_m_t) const
{
  MANIF_NOT_IMPLEMENTED_YET

  if (J_m_t)
  {
    (*J_m_t) = rjac();
  }

  return Manifold( A * x() - B * y(),
                   B * x() + A * y(),
                   cos_theta, sin_theta );
}

template <typename _Derived>
typename R2SO2TangentBase<_Derived>::Jacobian
R2SO2TangentBase<_Derived>::rjac() const
{
  Jacobian Jr = Jacobian::Identity();
  Jr.template topLeftCorner<2,2>() = rotation().transpose();

  return Jr;
}

template <typename _Derived>
typename R2SO2TangentBase<_Derived>::Jacobian
R2SO2TangentBase<_Derived>::ljac() const
{
  Jacobian Jl = Jacobian::Identity();

  Jl(0,2) =  y();
  Jl(1,2) = -x();

  return Jl;
}

template <typename _Derived>
typename R2SO2TangentBase<_Derived>::Jacobian
R2SO2TangentBase<_Derived>::adj() const
{
  Jacobian adj = Jacobian::Identity();
  adj.template topLeftCorner<2,2>() = rotation();
  adj(0,2) =  y();
  adj(1,2) = -x();
  return adj;
}

/// R2SO2Tangent specific API

template <typename _Derived>
typename R2SO2TangentBase<_Derived>::Scalar
R2SO2TangentBase<_Derived>::x() const
{
  return coeffs().x();
}

template <typename _Derived>
typename R2SO2TangentBase<_Derived>::Scalar
R2SO2TangentBase<_Derived>::y() const
{
  return coeffs().y();
}

template <typename _Derived>
typename R2SO2TangentBase<_Derived>::Scalar
R2SO2TangentBase<_Derived>::angle() const
{
  return coeffs().z();
}

} /* namespace manif */

#endif /* _MANIF_MANIF_R2SO2_BASE_H_ */
