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

  using BlockV = typename DataType::template FixedSegmentReturnType<3>::Type;
  using BlockW = typename DataType::template FixedSegmentReturnType<3>::Type;
  using ConstBlockV = typename DataType::template ConstFixedSegmentReturnType<3>::Type;
  using ConstBlockW = typename DataType::template ConstFixedSegmentReturnType<3>::Type;

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

  BlockV v();
  const ConstBlockV v() const;

  BlockW w();
  const ConstBlockW w() const;

//  Scalar x() const;
//  Scalar y() const;
//  Scalar z() const;

  //Scalar roll() const;
  //Scalar pitch() const;
  //Scalar yaw() const;

//protected:

  const Eigen::Map<const SO3Tangent<Scalar>> asSO3() const
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

  if (J_m_t)
  {
    *J_m_t = rjac();
  }

  /// @note Eq. 10.93
  return Manifold(asSO3().ljac()*v(), asSO3().retract().quat());
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

/// @note Eq. 10.95
/// @note barfoot14tro Eq. 102
template <typename _Derived>
typename SE3TangentBase<_Derived>::Jacobian
SE3TangentBase<_Derived>::rjac() const
{
  using std::cos;
  using std::sin;
  using std::sqrt;
//  MANIF_NOT_IMPLEMENTED_YET;

  /// @note Eq. 10.95
  Jacobian Jr = Jacobian::Zero();
  Jr.template topLeftCorner<3,3>() = asSO3().rjac();
  Jr.template bottomRightCorner<3,3>() =
      Jr.template topLeftCorner<3,3>();

  const Scalar theta_sq = asSO3().coeffs().squaredNorm();
  const LieType V = skew(v());
  const LieType W = asSO3().hat();

  Scalar A, B, C;

  // Small angle approximation
  if (theta_sq <= Constants<Scalar>::eps)
  {
    A = Scalar(1./6.);
    B = Scalar(1./24.);
    C = Scalar(1./24.) + Scalar(3./120.);
  }
  else
  {
   const Scalar theta     = sqrt(theta_sq);
   const Scalar sin_theta = sin(theta);
   const Scalar cos_theta = cos(theta);

    A = (theta - sin_theta) / (theta_sq*theta);
    B = ((Scalar(1) - theta_sq) / (Scalar(2) - cos_theta)) / (theta_sq*theta_sq);
    C = (((Scalar(1) - theta_sq) / (Scalar(2) - cos_theta)) / (theta_sq*theta_sq)) -
        Scalar(3)*(theta - sin_theta - theta_sq*theta/Scalar(6))/(theta_sq*theta_sq*theta);
  }

  Jr.template bottomLeftCorner<3,3>().noalias() =
      - Scalar(0.5) * V
      + A * (W*V + V*W - W*V*W)
      + B * (W*W*V + V*W*W - Scalar(3)*W*V*W)
      - C * Scalar(0.5) * (W*V*W*W + W*W*V*W);

    /////////////
//  double phi = w.norm();
//  if (fabs(phi)>1e-5) {
//    const double sinPhi = sin(phi), cosPhi = cos(phi);
//    const double phi2 = phi * phi, phi3 = phi2 * phi, phi4 = phi3 * phi, phi5 = phi4 * phi;
//    // Invert the sign of odd-order terms to have the right Jacobian
//    Q = -0.5*V + (phi-sinPhi)/phi3*(W*V + V*W - W*V*W)
//        + (1-phi2/2-cosPhi)/phi4*(W*W*V + V*W*W - 3*W*V*W)
//        - 0.5*((1-phi2/2-cosPhi)/phi4 - 3*(phi-sinPhi-phi3/6.)/phi5)*(W*V*W*W + W*W*V*W);
//  }
//  else {
//    Q = -0.5*V + 1./6.*(W*V + V*W - W*V*W)
//        + 1./24.*(W*W*V + V*W*W - 3*W*V*W)
//        - 0.5*(1./24. + 3./120.)*(W*V*W*W + W*W*V*W);
//  }

  return Jr;
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

  adj.template bottomLeftCorner<3,3>() = skew(v());

  return adj;
}

/// SE3Tangent specific API


template <typename _Derived>
typename SE3TangentBase<_Derived>::BlockV
SE3TangentBase<_Derived>::v()
{
  return coeffs().template head<3>();
}

template <typename _Derived>
const typename SE3TangentBase<_Derived>::ConstBlockV
SE3TangentBase<_Derived>::v() const
{
  return coeffs().template head<3>();
}

template <typename _Derived>
typename SE3TangentBase<_Derived>::BlockW SE3TangentBase<_Derived>::w()
{
  return coeffs().template tail<3>();
}

template <typename _Derived>
const typename SE3TangentBase<_Derived>::ConstBlockW
SE3TangentBase<_Derived>::w() const
{
  return coeffs().template tail<3>();
}

//template <typename _Derived>
//typename SE3TangentBase<_Derived>::Scalar
//SE3TangentBase<_Derived>::x() const
//{
//  return data()->x();
//}

//template <typename _Derived>
//typename SE3TangentBase<_Derived>::Scalar
//SE3TangentBase<_Derived>::y() const
//{
//  return data()->y();
//}

//template <typename _Derived>
//typename SE3TangentBase<_Derived>::Scalar
//SE3TangentBase<_Derived>::z() const
//{
//  return data()->z();
//}

} /* namespace manif */

#endif /* _MANIF_MANIF_SE3_BASE_H_ */
