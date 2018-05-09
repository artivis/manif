#ifndef _MANIF_MANIF_SO3_BASE_H_
#define _MANIF_MANIF_SO3_BASE_H_

#include "manif/impl/so3/SO3_properties.h"
#include "manif/impl/manifold_base.h"
#include "manif/impl/utils.h"

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

  SO3Base<_Derived>& setIdentity();

  Manifold inverse(OptJacobianRef J_minv_m = {}) const;
  Tangent lift(OptJacobianRef J_t_m = {}) const;

  template <typename _DerivedOther>
  Manifold compose(const ManifoldBase<_DerivedOther>& m,
                   OptJacobianRef J_mc_ma = {},
                   OptJacobianRef J_mc_mb = {}) const;

  Vector act(const Vector &v,
             OptJacobianRef J_vout_m = {},
             OptJacobianRef J_vout_v = {}) const;

  using Base::coeffs;
  using Base::coeffs_nonconst;
  MANIF_INHERIT_MANIFOLD_AUTO_API
  MANIF_INHERIT_MANIFOLD_OPERATOR

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
SO3Base<_Derived>&
SO3Base<_Derived>::setIdentity()
{
  coeffs_nonconst().setIdentity();
  return *this;
}

template <typename _Derived>
typename SO3Base<_Derived>::Manifold
SO3Base<_Derived>::inverse(OptJacobianRef J_minv_m) const
{
  if (J_minv_m)
  {
    *J_minv_m = -rotation();
  }

  /// @todo, conjugate doc :
  /// equal to the multiplicative inverse if
  /// the quaternion is normalized
  return Manifold(coeffs().conjugate());
}

template <typename _Derived>
typename SO3Base<_Derived>::Tangent
SO3Base<_Derived>::lift(OptJacobianRef J_t_m) const
{
  using std::sqrt;
  using std::atan2;

  if (J_t_m)
  {
    /// @todo
    MANIF_NOT_IMPLEMENTED_YET
  }

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
    const Scalar two_angle = (Scalar(2.0) * (cos_angle < Scalar(0.0))) ?
                                 atan2(-sin_angle, -cos_angle) :
                                 atan2( sin_angle,  cos_angle);

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
SO3Base<_Derived>::compose(
    const ManifoldBase<_DerivedOther>& m,
    OptJacobianRef J_mc_ma,
    OptJacobianRef J_mc_mb) const
{
  if (J_mc_ma)
    *J_mc_ma = coeffs().conjugate().matrix(); // R2.tr

  if (J_mc_mb)
    J_mc_mb->setIdentity();

  return Manifold(coeffs() * m.coeffs());
}

template <typename _Derived>
typename SO3Base<_Derived>::Vector
SO3Base<_Derived>::act(const Vector &v,
                       OptJacobianRef J_vout_m,
                       OptJacobianRef J_vout_v) const
{
  if (J_vout_m)
  {
    (*J_vout_m) = -rotation() * skew(v);
  }

  if (J_vout_v)
  {
    (*J_vout_v) = rotation();
  }

  return rotation() * v;
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
