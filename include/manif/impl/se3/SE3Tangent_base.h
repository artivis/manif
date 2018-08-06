#ifndef _MANIF_MANIF_SE3TANGENT_BASE_H_
#define _MANIF_MANIF_SE3TANGENT_BASE_H_

#include "manif/impl/se3/SE3_properties.h"
#include "manif/impl/tangent_base.h"
#include "manif/impl/so3/SO3Tangent_map.h"

namespace manif
{

///////////////
///         ///
/// Tangent ///
///         ///
///////////////

template <typename _Derived>
struct SE3TangentBase : TangentBase<_Derived>
{
private:

  using Base = TangentBase<_Derived>;
  using Type = SE3TangentBase<_Derived>;

public:

  MANIF_TANGENT_PROPERTIES
  MANIF_TANGENT_TYPEDEF
  MANIF_INHERIT_TANGENT_OPERATOR

  using Base::data;
  using Base::coeffs;

  /// Tangent common API

  void zero();
  void random();

  Manifold retract(OptJacobianRef J_m_t = {}) const;

  LieType hat() const;

  Jacobian rjac() const;
  Jacobian ljac() const;

  Jacobian adj() const;

  /// SE3Tangent specific API

  Scalar x() const;
  Scalar y() const;
  Scalar z() const;

  //Scalar roll() const;
  //Scalar pitch() const;
  //Scalar yaw() const;

//protected:

  Eigen::Map<const SO3Tangent<Scalar>> asSO3() const
  {
    return Eigen::Map<const SO3Tangent<Scalar>>(coeffs().data()+3);
  }

  Eigen::Map<SO3Tangent<Scalar>> asSO3()
  {
    return Eigen::Map<SO3Tangent<Scalar>>(coeffs.data()+3);
  }
};

template <typename _Derived>
void SE3TangentBase<_Derived>::zero()
{
  coeffs().setZero();
}

template <typename _Derived>
void SE3TangentBase<_Derived>::random()
{
  coeffs().setRandom();
}

template <typename _Derived>
typename SE3TangentBase<_Derived>::Manifold
SE3TangentBase<_Derived>::retract(OptJacobianRef J_m_t) const
{
  using std::sqrt;
  using std::cos;
  using std::sin;

  const Eigen::Matrix<Scalar,3,1>& v = coeffs().template head<3>();

  if (J_m_t)
  {
    MANIF_NOT_IMPLEMENTED_YET;
  }

  /// @note Eq. 10.93
  return Manifold(asSO3().ljac()*v, asSO3().retract().quat());
}

template <typename _Derived>
typename SE3TangentBase<_Derived>::LieType
SE3TangentBase<_Derived>::hat() const
{
  return (LieType() <<
    Scalar(0)           , Scalar(-coeffs()(5)), Scalar( coeffs()(4)), Scalar(coeffs()(0)),
    Scalar( coeffs()(5)), Scalar(0)           , Scalar(-coeffs()(3)), Scalar(coeffs()(1)),
    Scalar(-coeffs()(4)), Scalar( coeffs()(3)), Scalar(0)           , Scalar(coeffs()(2)),
    Scalar(0)           , Scalar(0)           , Scalar(0)           , Scalar(0)
          ).finished();
}

template <typename _Derived>
typename SE3TangentBase<_Derived>::Jacobian
SE3TangentBase<_Derived>::rjac() const
{
  MANIF_NOT_IMPLEMENTED_YET;

  /// @note Eq. 10.95
  Jacobian Jr = Jacobian::Zero();
  Jr.template topLeftCorner<3,3>() = asSO3().rjac();
  Jr.template bottomRightCorner<3,3>() =
      Jr.template topLeftCorner<3,3>();

//  Jr.template bottomLeftCorner<3,3>() = /** @todo */(asSO3().ljac()*coeffs().template head<3>());

  return Jacobian::Constant(Scalar(1));
}

template <typename _Derived>
typename SE3TangentBase<_Derived>::Jacobian
SE3TangentBase<_Derived>::ljac() const
{
  MANIF_NOT_IMPLEMENTED_YET
  return Jacobian::Constant(Scalar(1));
}

template <typename _Derived>
typename SE3TangentBase<_Derived>::Jacobian
SE3TangentBase<_Derived>::adj() const
{
  Jacobian adj = Jacobian::Zero();
  adj.template topLeftCorner<3,3>() = asSO3().hat();
  adj.template bottomRightCorner<3,3>() =
      adj.template topLeftCorner<3,3>();

  adj.template bottomLeftCorner<3,3>()(0,1) = -coeffs()(2);
  adj.template bottomLeftCorner<3,3>()(0,2) =  coeffs()(1);

  adj.template bottomLeftCorner<3,3>()(1,0) =  coeffs()(2);
  adj.template bottomLeftCorner<3,3>()(1,2) = -coeffs()(0);

  adj.template bottomLeftCorner<3,3>()(2,0) = -coeffs()(1);
  adj.template bottomLeftCorner<3,3>()(2,1) =  coeffs()(0);

  return adj;
}

/// SE3Tangent specific API

template <typename _Derived>
typename SE3TangentBase<_Derived>::Scalar
SE3TangentBase<_Derived>::x() const
{
  return data()->x();
}

template <typename _Derived>
typename SE3TangentBase<_Derived>::Scalar
SE3TangentBase<_Derived>::y() const
{
  return data()->y();
}

template <typename _Derived>
typename SE3TangentBase<_Derived>::Scalar
SE3TangentBase<_Derived>::z() const
{
  return data()->z();
}

} /* namespace manif */

#endif /* _MANIF_MANIF_SE3_BASE_H_ */
