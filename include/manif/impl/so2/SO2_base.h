#ifndef _MANIF_MANIF_SO2_BASE_H_
#define _MANIF_MANIF_SO2_BASE_H_

#include "manif/impl/so2/SO2_properties.h"
#include "manif/impl/manifold_base.h"

namespace manif
{

////////////////
///          ///
/// Manifold ///
///          ///
////////////////

template <typename _Derived>
struct SO2Base : ManifoldBase<_Derived>
{
private:

  using Base = ManifoldBase<_Derived>;
  using Type = SO2Base<_Derived>;

public:

  static constexpr int Dim = internal::ManifoldProperties<Type>::Dim;
  static constexpr int DoF = internal::ManifoldProperties<Type>::DoF;
  static constexpr int N   = internal::ManifoldProperties<Type>::N;

  using Scalar   = typename Base::Scalar;

  using Manifold = typename Base::Manifold;
  using Tangent  = typename Base::Tangent;
  using Jacobian = typename Base::Jacobian;

  using DataType = typename Base::DataType;

  using Transformation = typename Base::Transformation;
  using Rotation = typename Base::Rotation;

  /// Manifold common API

  Transformation transform() const;
  Rotation rotation() const;

  void identity();

  Manifold inverse() const;
  Tangent lift() const;

  template <typename _DerivedOther>
  Manifold compose(const ManifoldBase<_DerivedOther>& m) const;

  using Base::coeffs;
  using Base::random;
  using Base::rplus;
  using Base::lplus;
  using Base::rminus;
  using Base::lminus;
  using Base::operator=;

  /// with Jacs

  void inverse(Manifold& m, Jacobian& j) const;

  template <typename _DerivedOther>
  void lift(TangentBase<_DerivedOther>& t, Jacobian& J_t_m) const;

  template <typename _DerivedOther0, typename _DerivedOther1>
  void compose(const ManifoldBase<_DerivedOther0>& mb,
               ManifoldBase<_DerivedOther1>& mout,
               Jacobian& J_c_a, Jacobian& J_c_b) const;

  /// SO2 specific functions

  /*const*/ Scalar/*&*/ real() const;
  /*const*/ Scalar/*&*/ imag() const;
  Scalar angle() const;

protected:

  using Base::coeffs_nonconst;

  /// @todo given a Eigen::Map<const SO2>
  /// coeffs()->x() return a reference to
  /// temporary ...

//  Scalar& real();
//  Scalar& imag();
};

template <typename _Derived>
typename SO2Base<_Derived>::Transformation
SO2Base<_Derived>::transform() const
{
  Transformation T(Transformation::Identity());
  T.template block<2,2>(0,0) = rotation();
  return T;
}

template <typename _Derived>
typename SO2Base<_Derived>::Rotation
SO2Base<_Derived>::rotation() const
{
  using std::sin;
  using std::cos;
  const Scalar theta = angle();
  return (Rotation() << cos(theta), -sin(theta),
                        sin(theta),  cos(theta)).finished();
}

template <typename _Derived>
void SO2Base<_Derived>::identity()
{
//  real() = 1;
//  imag() = 0;
  coeffs_nonconst().setIdentity();
}

template <typename _Derived>
typename SO2Base<_Derived>::Manifold
SO2Base<_Derived>::inverse() const
{
  return Manifold(real(), -imag());
}

template <typename _Derived>
typename SO2Base<_Derived>::Tangent
SO2Base<_Derived>::lift() const
{
  return Tangent(angle());
}

template <typename _Derived>
template <typename _DerivedOther>
typename SO2Base<_Derived>::Manifold
SO2Base<_Derived>::compose(const ManifoldBase<_DerivedOther>& m) const
{
  static_assert(
    std::is_base_of<SO2Base<_DerivedOther>, _DerivedOther>::value,
    "Argument does not inherit from SE2Base !");

  const auto& m_so2 = static_cast<const SO2Base<_DerivedOther>&>(m);

  const Scalar& lhs_real = real();
  const Scalar& lhs_imag = imag();
  const Scalar& rhs_real = m_so2.real();
  const Scalar& rhs_imag = m_so2.imag();

  return Manifold(
        lhs_real * rhs_real - lhs_imag * rhs_imag,
        lhs_real * rhs_imag + lhs_imag * rhs_real
        );
}

/// with Jacs

template <typename _Derived>
void SO2Base<_Derived>::inverse(Manifold& m, Jacobian& J) const
{
  m = inverse();
  J.setConstant(-1);
}

template <typename _Derived>
template <typename _DerivedOther>
void SO2Base<_Derived>::lift(TangentBase<_DerivedOther>& t,
                             Jacobian& J_t_m) const
{
  t = lift();
  J_t_m.setConstant(1);
}

template <typename _Derived>
template <typename _DerivedOther0, typename _DerivedOther1>
void SO2Base<_Derived>::compose(const ManifoldBase<_DerivedOther0>& mb,
                                ManifoldBase<_DerivedOther1>& mout,
                                Jacobian& J_c_a,
                                Jacobian& J_c_b) const
{
  mout = compose(mb);
  J_c_a.setConstant(1);
  J_c_b.setConstant(1);
}

/// SO2 specific function

template <typename _Derived>
/*const*/ typename SO2Base<_Derived>::Scalar/*&*/
SO2Base<_Derived>::real() const
{
  return coeffs().x();
}

template <typename _Derived>
/*const*/ typename SO2Base<_Derived>::Scalar/*&*/
SO2Base<_Derived>::imag() const
{
  return coeffs().y();
}

template <typename _Derived>
typename SO2Base<_Derived>::Scalar
SO2Base<_Derived>::angle() const
{
  using std::atan2;
  return atan2(imag(), real());
}

//template <typename _Derived>
//typename SO2Base<_Derived>::Scalar&
//SO2Base<_Derived>::real()
//{
//  return coeffs.x();
//}

//template <typename _Derived>
//typename SO2Base<_Derived>::Scalar&
//SO2Base<_Derived>::imag()
//{
//  return coeffs.y();
//}

} /* namespace manif */

#endif /* _MANIF_MANIF_SO2_BASE_H_ */
