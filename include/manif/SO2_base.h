#ifndef _MANIF_MANIF_SO2_BASE_H_
#define _MANIF_MANIF_SO2_BASE_H_

#include "manif/manifold_base.h"
#include "manif/tangent_base.h"

namespace manif
{

template <typename _Derived> struct SO2Base;
template <typename _Derived> struct SO2TangentBase;

template <>
template <typename _Derived>
struct ManifoldProperties<SO2Base<_Derived>>
{
  static constexpr int Dim = 2;
  static constexpr int DoF = 1;
};

////////////////
///          ///
/// Manifold ///
///          ///
////////////////

template <typename _Derived>
struct SO2Base : ManifoldBase<_Derived>
{
  using Base = ManifoldBase<_Derived>;

  using Scalar = typename Base::Scalar;

  using Manifold = typename Base::Manifold;
  using Tangent  = typename Base::Tangent;

  using JacobianType = typename Base::JacobianType;

  using ManifoldDataType = typename Base::ManifoldDataType;

  static constexpr int Dim = ManifoldProperties<SO2Base<_Derived>>::Dim;
  static constexpr int DoF = ManifoldProperties<SO2Base<_Derived>>::DoF;

  using Transformation  = typename Base::Transformation;

  using Base::data;

  Transformation matrix() const;

  void identity();

  void random();

  Manifold inverse() const;

  Manifold rplus(const Tangent& /*t*/) const;

  Manifold lplus(const Tangent& /*t*/) const;

  Manifold rminus(const Manifold& /*m*/) const;

  Manifold lminus(const Manifold& /*m*/) const;

  Tangent lift() const;

  Manifold compose(const Manifold& /*m*/) const;

  /// with Jacs

  void inverse(Manifold& m, JacobianType& j) const;
};

template <typename _Derived>
typename SO2Base<_Derived>::Transformation
SO2Base<_Derived>::matrix() const
{
  return Transformation();
}

template <typename _Derived>
void SO2Base<_Derived>::identity()
{
  MANIF_INFO("SO2Base identity");
  data()->setIdentity();
}

template <typename _Derived>
void SO2Base<_Derived>::random()
{
  MANIF_INFO("SO2Base random");
  data()->setRandom();
}

template <typename _Derived>
typename SO2Base<_Derived>::Manifold
SO2Base<_Derived>::inverse() const
{
  MANIF_INFO("SO2Base inverse");
  return Manifold(
        data()->cwiseProduct(ManifoldDataType(1,-1))
        );
}

template <typename _Derived>
typename SO2Base<_Derived>::Manifold
SO2Base<_Derived>::rplus(const Tangent& t) const
{
  /// @todo check this
  return Manifold( Tangent(lift().data()->x() + t.data()->x()).retract() );
}

template <typename _Derived>
typename SO2Base<_Derived>::Manifold
SO2Base<_Derived>::lplus(const Tangent& t) const
{
  // In SO2 rotation are commutative
  return rplus(t);
}

template <typename _Derived>
typename SO2Base<_Derived>::Tangent
SO2Base<_Derived>::lift() const
{
  MANIF_INFO("SO2Base lift");
  using std::atan2;
  return Tangent(
        typename Tangent::TangentDataType(
          atan2(data()->y(),
                data()->x())));
}

template <typename _Derived>
typename SO2Base<_Derived>::Manifold
SO2Base<_Derived>::compose(const Manifold& m) const
{
  const Scalar& lhs_real = data()->x();
  const Scalar& lhs_imag = data()->y();
  const Scalar& rhs_real = m.data()->x();
  const Scalar& rhs_imag = m.data()->y();

  return Manifold(
        typename Manifold::ManifoldDataType(
          lhs_real * rhs_real - lhs_imag * rhs_imag,
          lhs_real * rhs_imag + lhs_imag * rhs_real
          ) );
}

/// with Jacs

template <typename _Derived>
void SO2Base<_Derived>::inverse(Manifold& m, JacobianType& j) const
{
  using std::cos;
  using std::sin;
  const Scalar theta = m.lift().data()->x();
  m = inverse();
  j << cos(theta), -sin(theta),
       sin(theta),  cos(theta);
}

///////////////
///         ///
/// Tangent ///
///         ///
///////////////

template <typename _Derived>
struct SO2TangentBase : TangentBase<_Derived>
{
  static constexpr int Dim = ManifoldProperties<SO2Base<_Derived>>::Dim;
  static constexpr int DoF = ManifoldProperties<SO2Base<_Derived>>::DoF;

  using Base = TangentBase<_Derived>;

  using Manifold = typename Base::Manifold;
  using Tangent  = typename Base::Tangent;

  using TangentDataType  = typename Base::TangentDataType;

  using Base::data;

  void zero();
  void random();
  Manifold retract() const;
};

template <typename _Derived>
void SO2TangentBase<_Derived>::zero()
{
  MANIF_INFO("SO2TangentBase zero");
  data()->setZero();
}

template <typename _Derived>
void SO2TangentBase<_Derived>::random()
{
  MANIF_INFO("SO2TangentBase random");
  data()->setRandom();
}

template <typename _Derived>
typename SO2TangentBase<_Derived>::Manifold
SO2TangentBase<_Derived>::retract() const
{
  MANIF_INFO("SO2TangentBase random");
  using std::cos;
  using std::sin;
  return Manifold(
        typename Manifold::ManifoldDataType(cos(data()->operator()(0)),
                                            sin(data()->operator()(0)))
        );
}

} /* namespace manif */

#endif /* _MANIF_MANIF_SO2_BASE_H_ */
