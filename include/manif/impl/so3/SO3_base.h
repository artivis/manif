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

  MANIF_MANIFOLD_TYPEDEF

  using QuaternionDataType = Eigen::Quaternion<Scalar>;

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

  Jacobian adj() const;

  using Base::coeffs;
  MANIF_INHERIT_MANIFOLD_AUTO_API
  MANIF_INHERIT_MANIFOLD_OPERATOR

  /// SO3 specific functions

  Scalar x() const;
  Scalar y() const;
  Scalar z() const;
  Scalar w() const;

  QuaternionDataType quat() const
  {
    return QuaternionDataType(coeffs());
  }

protected:

  using Base::coeffs_nonconst;

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
  return quat().matrix();
}

template <typename _Derived>
SO3Base<_Derived>&
SO3Base<_Derived>::setIdentity()
{
  coeffs_nonconst().setZero();
  coeffs_nonconst()(3) = Scalar(1);
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
  return Manifold(quat().conjugate());
}

template <typename _Derived>
typename SO3Base<_Derived>::Tangent
SO3Base<_Derived>::lift(OptJacobianRef J_t_m) const
{
  using std::sqrt;
  using std::atan2;

  Tangent tan;
  Scalar lift_coeff;

  const Scalar sin_angle_squared = coeffs().template head<3>().squaredNorm();
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
    const Scalar two_angle = Scalar(2.0) * ((cos_angle < Scalar(0.0)) ?
                                 atan2(-sin_angle, -cos_angle) :
                                 atan2( sin_angle,  cos_angle));

    lift_coeff = two_angle / sin_angle;
  }
  else
  {
    // small-angle approximation
    lift_coeff = Scalar(2.0);
  }

  tan = Tangent(coeffs().template head<3>() * lift_coeff);

//  using std::atan2;
//  Scalar n = coeffs().template head<3>().norm();
//  Scalar angle(0);
//  typename Tangent::DataType axis(1,0,0);
//  if (n<Constants<Scalar>::eps)
//    n = coeffs().template head<3>().stableNorm();
//  if (n > Scalar(0))
//  {
//    angle = Scalar(2)*atan2(n, w());
//    axis  = coeffs().template head<3>() / n;
//  }

//  tan = Tangent(axis*angle);

  if (J_t_m)
  {
    Scalar theta2 = tan.coeffs().squaredNorm();
    typename Tangent::LieType W = tan.hat();
    if (theta2 <= Constants<Scalar>::eps)
      (*J_t_m) = Jacobian::Identity() + Scalar(0.5) * W; // Small angle approximation
    else
    {
      Scalar theta = sqrt(theta2);  // rotation angle
      Jacobian M;
      M.noalias() = (Scalar(1) / theta2 - (Scalar(1) + cos(theta)) / (Scalar(2) * theta * sin(theta))) * (W * W);
      (*J_t_m) = Jacobian::Identity() + Scalar(0.5) * W + M; //is this really more optimized?
    }
  }

  return tan;
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
    *J_mc_ma = m.rotation().transpose();

  if (J_mc_mb)
    J_mc_mb->setIdentity();

  return Manifold(QuaternionDataType(coeffs()) *
                  QuaternionDataType(m.coeffs()));
}

template <typename _Derived>
typename SO3Base<_Derived>::Vector
SO3Base<_Derived>::act(const Vector &v,
                       OptJacobianRef J_vout_m,
                       OptJacobianRef J_vout_v) const
{
  if (J_vout_m)
  {
    (*J_vout_m) = -rotation() * hat(v);
  }

  if (J_vout_v)
  {
    (*J_vout_v) = rotation();
  }

  return rotation() * v;
}

template <typename _Derived>
typename SO3Base<_Derived>::Jacobian
SO3Base<_Derived>::adj() const
{
  return rotation();
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
