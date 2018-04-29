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

  static constexpr int Dim = internal::ManifoldProperties<Type>::Dim;
  static constexpr int DoF = internal::ManifoldProperties<Type>::DoF;
  static constexpr int N   = internal::ManifoldProperties<Type>::N;

  using Scalar = typename Base::Scalar;

  using Manifold = typename Base::Manifold;
  using Tangent  = typename Base::Tangent;

  using Jacobian = typename Base::Jacobian;

  using DataType = typename Base::DataType;

  using Transformation  = typename Base::Transformation;
  using Rotation = typename Base::Rotation;

  using Base::data;
  using Base::operator =;

  /// Manifold common API

  Transformation transform() const;
  Rotation rotation() const;

  void identity();
  void random();

  Manifold inverse() const;
  Tangent lift() const;

  template <typename _DerivedOther>
  Manifold compose(const ManifoldBase<_DerivedOther>& m) const;

  using Base::rplus;
  using Base::lplus;
  using Base::rminus;
  using Base::lminus;

  /// with Jacs

  void inverse(Manifold& m, Jacobian& j) const;
  void lift(const Manifold& m, Tangent& t, Jacobian& J_t_m) const;

  void compose(const Manifold& mb,
               Manifold& mout,
               Jacobian& J_c_a, Jacobian& J_c_b) const;

  /// SO3 specific functions

  Scalar x() const;
  Scalar y() const;
  Scalar z() const;
  Scalar w() const;
};

template <typename _Derived>
typename SO3Base<_Derived>::Transformation
SO3Base<_Derived>::transform() const
{
  MANIF_NOT_IMPLEMENTED_YET
  return Transformation();
}

template <typename _Derived>
typename SO3Base<_Derived>::Rotation
SO3Base<_Derived>::rotation() const
{
  MANIF_NOT_IMPLEMENTED_YET
  return Rotation();
}

template <typename _Derived>
void SO3Base<_Derived>::identity()
{
  data()->setIdentity();
}

template <typename _Derived>
void SO3Base<_Derived>::random()
{
  const auto m = Tangent::Random().retract();

  *data() = *m.data();

  /// @todo the following does not work
  /// figure out why
//  Base::operator =(Tangent::Random().retract());
//  *this = Tangent::Random().retract();
}

template <typename _Derived>
typename SO3Base<_Derived>::Manifold
SO3Base<_Derived>::inverse() const
{
  /// @todo, conjugate doc :
  /// equal to the multiplicative inverse if
  /// the quaternion is normalized
  return Manifold(data()->conjugate());
}

template <typename _Derived>
typename SO3Base<_Derived>::Tangent
SO3Base<_Derived>::lift() const
{
  using std::sqrt;
  using std::atan2;

  const Scalar sin_angle_squared = data()->vec().squaredNorm();
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
    return Tangent(data()->vec() * k);
  }
  else
  {
    // small-angle approximation
    return Tangent(data()->vec() * Scalar(2.0));
  }
}

template <typename _Derived>
template <typename _DerivedOther>
typename SO3Base<_Derived>::Manifold
SO3Base<_Derived>::compose(const ManifoldBase<_DerivedOther>& m) const
{
  return Manifold(data()->operator *(*m.data()));
}

/// with Jacs

template <typename _Derived>
void SO3Base<_Derived>::inverse(Manifold& m, Jacobian& J) const
{
  MANIF_NOT_IMPLEMENTED_YET;
  m = inverse();
  J = rotation();
}

template <typename _Derived>
void SO3Base<_Derived>::lift(const Manifold& m,
                             Tangent& t,
                             Jacobian& J_t_m) const
{
  MANIF_NOT_IMPLEMENTED_YET;
  m = lift();
}

template <typename _Derived>
void SO3Base<_Derived>::compose(const Manifold& mb,
                                Manifold& mout,
                                Jacobian& J_c_a,
                                Jacobian& J_c_b) const
{
  mout = compose(mb);
  J_c_a = data()->conjugate()->matrix(); // R2.tr
  J_c_b.setIdentity();
}

/// SO3 specific

template <typename _Derived>
typename SO3Base<_Derived>::Scalar
SO3Base<_Derived>::x() const
{
  return data()->x();
}

template <typename _Derived>
typename SO3Base<_Derived>::Scalar
SO3Base<_Derived>::y() const
{
  return data()->y();
}

template <typename _Derived>
typename SO3Base<_Derived>::Scalar
SO3Base<_Derived>::z() const
{
  return data()->z();
}

template <typename _Derived>
typename SO3Base<_Derived>::Scalar
SO3Base<_Derived>::w() const
{
  return data()->w();
}

} /* namespace manif */

#endif /* _MANIF_MANIF_SO3_BASE_H_ */
