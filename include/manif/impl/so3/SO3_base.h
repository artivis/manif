#ifndef _MANIF_MANIF_SO3_BASE_H_
#define _MANIF_MANIF_SO3_BASE_H_

#include "manif/impl/so3/SO3_properties.h"
#include "manif/impl/manifold_base.h"

namespace manif
{

////////////////
///          ///
/// Manifold ///
///          ///
////////////////

template <typename _Derived>
struct SO3Base : ManifoldBase<_Derived>
{
private:

  using Base = ManifoldBase<_Derived>;
  using Type = SO3Base<_Derived>;

public:

  MANIF_MANIFOLD_PROPERTIES
  MANIF_MANIFOLD_TYPEDEF

  /// Manifold common API

  Transformation transform() const;
  Rotation rotation() const;

  void identity();

  Manifold inverse() const;
  Tangent lift() const;

  template <typename _DerivedOther>
  Manifold compose(const ManifoldBase<_DerivedOther>& m) const;

  using Base::coeffs;
  using Base::coeffs_nonconst;
  MANIF_INHERIT_MANIFOLD_AUTO_API
  MANIF_INHERIT_MANIFOLD_OPERATOR

  /// with Jacs

  void inverse(Manifold& minv, Jacobian& J_minv_m) const;
  void lift(Tangent& t, Jacobian& J_t_m) const;

  void compose(const Manifold& mb,
               Manifold& mout,
               Jacobian& J_c_a, Jacobian& J_c_b) const;

  /// SO3 specific functions

  Scalar x() const;
  Scalar y() const;
  Scalar z() const;
  Scalar w() const;

protected:

  void normalize();
};

template <typename _Derived>
typename SO3Base<_Derived>::Transformation
SO3Base<_Derived>::transform() const
{
  Transformation T = Transformation::Identity();
  T.template block<3,3>(0,0) = rotation();
  return T;
}

template <typename _Derived>
typename SO3Base<_Derived>::Rotation
SO3Base<_Derived>::rotation() const
{
  return coeffs().matrix();
}

template <typename _Derived>
void SO3Base<_Derived>::identity()
{
  coeffs_nonconst().setIdentity();
}

template <typename _Derived>
typename SO3Base<_Derived>::Manifold
SO3Base<_Derived>::inverse() const
{
  /// @todo, conjugate doc :
  /// equal to the multiplicative inverse if
  /// the quaternion is normalized
  return Manifold(coeffs().conjugate());
}

template <typename _Derived>
typename SO3Base<_Derived>::Tangent
SO3Base<_Derived>::lift() const
{
  using std::sqrt;
  using std::atan2;

  const Scalar sin_angle_squared = coeffs().vec().squaredNorm();
  if (sin_angle_squared > Constants<Scalar>::eps)
  {
    const Scalar sin_angle = sqrt(sin_angle_squared);
    const Scalar cos_angle = w();

    /** @note If (cos_angle < 0) then angle >= pi/2 ,
     *  means : angle for angle_axis vector >= pi (== 2*angle)
     *   |-> results in correct rotation but not a normalized angle_axis vector
     *
     * In that case we observe that 2 * angle ~ 2 * angle - 2 * pi,
     * which is equivalent saying
     *
     * angle - pi = atan(sin(angle - pi), cos(angle - pi))
     *            = atan(-sin(angle), -cos(angle))
     */
    const Scalar two_angle = (Scalar(2.0) * ((cos_angle < Scalar(0.0))) ?
                                 atan2(-sin_angle, -cos_angle) :
                                 atan2( sin_angle,  cos_angle));

    const Scalar k = two_angle / sin_angle;
    return Tangent(coeffs().vec() * k);
  }
  else
  {
    // small-angle approximation
    return Tangent(coeffs().vec() * Scalar(2.0));
  }
}

template <typename _Derived>
template <typename _DerivedOther>
typename SO3Base<_Derived>::Manifold
SO3Base<_Derived>::compose(const ManifoldBase<_DerivedOther>& m) const
{
  return Manifold(coeffs() * m.coeffs());
}

/// with Jacs

template <typename _Derived>
void SO3Base<_Derived>::inverse(Manifold& minv, Jacobian& J_minv_m) const
{
  minv = inverse();
  J_minv_m = -rotation();
}

template <typename _Derived>
void SO3Base<_Derived>::lift(Tangent& t,
                             Jacobian& J_t_m) const
{
  t = lift();
  /// @todo
//  J_t_m
}

template <typename _Derived>
void SO3Base<_Derived>::compose(const Manifold& mb,
                                Manifold& mout,
                                Jacobian& J_c_a,
                                Jacobian& J_c_b) const
{
  mout = compose(mb);
  J_c_a = coeffs().conjugate().matrix(); // R2.tr
  J_c_b.setIdentity();
}

/// SO3 specific

template <typename _Derived>
typename SO3Base<_Derived>::Scalar
SO3Base<_Derived>::x() const
{
  return coeffs().x();
}

template <typename _Derived>
typename SO3Base<_Derived>::Scalar
SO3Base<_Derived>::y() const
{
  return coeffs().y();
}

template <typename _Derived>
typename SO3Base<_Derived>::Scalar
SO3Base<_Derived>::z() const
{
  return coeffs().z();
}

template <typename _Derived>
typename SO3Base<_Derived>::Scalar
SO3Base<_Derived>::w() const
{
  return coeffs().w();
}

template <typename _Derived>
void SO3Base<_Derived>::normalize()
{
  coeffs().normalize();
}

} /* namespace manif */

#endif /* _MANIF_MANIF_SO3_BASE_H_ */
