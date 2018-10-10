#ifndef _MANIF_MANIF_R3SO3TANGENT_BASE_H_
#define _MANIF_MANIF_R3SO3TANGENT_BASE_H_

#include "manif/impl/R3SO3/R3SO3_properties.h"
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
struct R3SO3TangentBase : TangentBase<_Derived>
{
private:

  using Base = TangentBase<_Derived>;
  using Type = R3SO3TangentBase<_Derived>;

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

  LieAlg hat() const;

  Jacobian rjac() const;
  Jacobian ljac() const;

  Jacobian adj() const;

  /// R3SO3Tangent specific API

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
void R3SO3TangentBase<_Derived>::zero()
{
  coeffs().setZero();
}

template <typename _Derived>
void R3SO3TangentBase<_Derived>::random()
{
  coeffs().setRandom();
}

template <typename _Derived>
typename R3SO3TangentBase<_Derived>::Manifold
R3SO3TangentBase<_Derived>::retract(OptJacobianRef J_m_t) const
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
typename R3SO3TangentBase<_Derived>::LieAlg
R3SO3TangentBase<_Derived>::hat() const
{
  return (LieAlg() <<
    Scalar(0)           , Scalar(-coeffs()(5)), Scalar( coeffs()(4)), Scalar(coeffs()(0)),
    Scalar( coeffs()(5)), Scalar(0)           , Scalar(-coeffs()(3)), Scalar(coeffs()(1)),
    Scalar(-coeffs()(4)), Scalar( coeffs()(3)), Scalar(0)           , Scalar(coeffs()(2)),
    Scalar(0)           , Scalar(0)           , Scalar(0)           , Scalar(0)
          ).finished();
}

/// @note Eq. 10.97
template <typename _Derived>
typename R3SO3TangentBase<_Derived>::Jacobian
R3SO3TangentBase<_Derived>::rjac() const
{
  Jacobian Jr = Jacobian::Zero();
  Jr.template topLeftCorner<3,3>() = asSO3().rjac();
  Jr.template bottomRightCorner<3,3>() = asSO3().rotation().tranpose();

  return Jr;
}

/// @note Eq. 10.97
template <typename _Derived>
typename R3SO3TangentBase<_Derived>::Jacobian
R3SO3TangentBase<_Derived>::ljac() const
{
  Jacobian Jl = Jacobian::Identity();
  Jl.template topLeftCorner<3,3>() = asSO3().ljac();
  Jl.template bottomLeftCorner<3,3>() =
      skew(t()) * Jl.template topLeftCorner<3,3>();

  return Jl;
}

template <typename _Derived>
typename R3SO3TangentBase<_Derived>::Jacobian
R3SO3TangentBase<_Derived>::adj() const
{
  Jacobian adj = Jacobian::Zero();
  adj.template topLeftCorner<3,3>() = asSO3().hat();
  adj.template bottomRightCorner<3,3>() =
      adj.template topLeftCorner<3,3>();

  adj.template bottomLeftCorner<3,3>() = skew(v());

  return adj;
}

/// R3SO3Tangent specific API

template <typename _Derived>
typename R3SO3TangentBase<_Derived>::BlockV
R3SO3TangentBase<_Derived>::v()
{
  return coeffs().template head<3>();
}

template <typename _Derived>
const typename R3SO3TangentBase<_Derived>::ConstBlockV
R3SO3TangentBase<_Derived>::v() const
{
  return coeffs().template head<3>();
}

template <typename _Derived>
typename R3SO3TangentBase<_Derived>::BlockW R3SO3TangentBase<_Derived>::w()
{
  return coeffs().template tail<3>();
}

template <typename _Derived>
const typename R3SO3TangentBase<_Derived>::ConstBlockW
R3SO3TangentBase<_Derived>::w() const
{
  return coeffs().template tail<3>();
}

} /* namespace manif */

#endif /* _MANIF_MANIF_R3SO3_BASE_H_ */
