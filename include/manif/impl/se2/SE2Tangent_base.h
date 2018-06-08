#ifndef _MANIF_MANIF_SE2TANGENT_BASE_H_
#define _MANIF_MANIF_SE2TANGENT_BASE_H_

#include "manif/impl/se2/SE2_properties.h"
#include "manif/impl/tangent_base.h"

namespace manif
{

///////////////
///         ///
/// Tangent ///
///         ///
///////////////

template <typename _Derived>
struct SE2TangentBase : TangentBase<_Derived>
{
private:

  using Base = TangentBase<_Derived>;
  using Type = SE2TangentBase<_Derived>;

public:

  MANIF_TANGENT_PROPERTIES
  MANIF_TANGENT_TYPEDEF
  MANIF_INHERIT_TANGENT_OPERATOR

  using Base::data;
  using Base::coeffs;

  /// Tangent common API

  LieType skew() const;

  Manifold retract(OptJacobianRef J_m_t = {}) const;

  /// SE2Tangent specific API

  Scalar x() const;
  Scalar y() const;
  Scalar angle() const;
};

template <typename _Derived>
typename SE2TangentBase<_Derived>::LieType
SE2TangentBase<_Derived>::skew() const
{
  return ( LieType() <<
             Scalar(0), -angle(),   x(),
             angle(),    Scalar(0), y(),
             Scalar(0),  Scalar(0), Scalar(0) ).finished();
}

template <typename _Derived>
typename SE2TangentBase<_Derived>::Manifold
SE2TangentBase<_Derived>::retract(OptJacobianRef J_m_t) const
{
  using std::abs;
  using std::cos;
  using std::sin;

  const Scalar theta = angle();
  const Scalar cos_theta = cos(theta);
  const Scalar sin_theta = sin(theta);

  Scalar A,  // sin_theta_by_theta
         B;  // one_minus_cos_theta_by_theta

  if (abs(theta) < Constants<Scalar>::eps)
  {
    // Taylor approximation
    const Scalar theta_sq = theta * theta;
    A = Scalar(1) - Scalar(1. / 6.) * theta_sq;
    B = Scalar(.5) * theta - Scalar(1. / 24.) * theta * theta_sq;
  }
  else
  {
    // Euler
    A = sin_theta / theta;
    B = (Scalar(1) - cos_theta) / theta;
  }

  Manifold ret( A * x() - B * y(),
                B * x() + A * y(),
                cos_theta, sin_theta );

  if (J_m_t)
  {
//    Jacobian rjac = Jacobian::Identity();
//    rjac.template topLeftCorner<2,2>() =
//        ret.rotation().template transpose();

//    (*J_m_t) = rjac;

    // Jr
    J_m_t->setIdentity();
    (*J_m_t)(0,0) =  cos_theta;
    (*J_m_t)(0,1) =  sin_theta;
    (*J_m_t)(1,0) = -sin_theta;
    (*J_m_t)(1,1) =  cos_theta;

//    std::cout << "se2 Jr:\n" << (*J_m_t) << "\n";
  }

  return ret;
}

/// SE2Tangent specific API

template <typename _Derived>
typename SE2TangentBase<_Derived>::Scalar
SE2TangentBase<_Derived>::x() const
{
  return coeffs().x();
}

template <typename _Derived>
typename SE2TangentBase<_Derived>::Scalar
SE2TangentBase<_Derived>::y() const
{
  return coeffs().y();
}

template <typename _Derived>
typename SE2TangentBase<_Derived>::Scalar
SE2TangentBase<_Derived>::angle() const
{
  return coeffs().z();
}

} /* namespace manif */

#endif /* _MANIF_MANIF_SE2_BASE_H_ */
