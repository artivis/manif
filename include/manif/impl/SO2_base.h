#ifndef _MANIF_MANIF_SO2_BASE_H_
#define _MANIF_MANIF_SO2_BASE_H_

#include "manif/impl/SO2_properties.h"
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

  using Scalar = typename Base::Scalar;

  using Manifold = typename Base::Manifold;
  using Tangent  = typename Base::Tangent;

  using Jacobian = typename Base::Jacobian;

  using ManifoldDataType = typename Base::ManifoldDataType;

  using Transformation  = typename Base::Transformation;
  using Rotation = typename Base::Rotation;

  using Base::data;

  /// Manifold common API

  Transformation matrix() const;
  Rotation rotation() const;

  void identity();
  void random();

  Manifold inverse() const;

  using Base::rplus;
  using Base::lplus;
  using Base::rminus;
  using Base::lminus;
/*
  Manifold rplus(const Tangent& t) const;
  Manifold lplus(const Tangent& t) const;

  Tangent rminus(const Manifold& m) const;
  Tangent lminus(const Manifold& m) const;
*/
  Tangent lift() const;

  Manifold compose(const Manifold& m) const;

  /// with Jacs

  void inverse(Manifold& m, Jacobian& j) const;

  void rplus(const Tangent& t, Manifold& m,
             Jacobian& J_c_a, Jacobian& J_c_b) const;

  void lplus(const Tangent& t, Manifold& m,
             Jacobian& J_c_a, Jacobian& J_c_b) const;

  void rminus(const Manifold& min, Manifold& mout,
              Jacobian& J_c_a, Jacobian& J_c_b) const;

  void lminus(const Manifold& min, Manifold& mout,
              Jacobian& J_c_a, Jacobian& J_c_b) const;

  void lift(const Manifold& m, Tangent& t, Jacobian& J_t_m) const;

  void compose(const Manifold& ma, const Manifold& mb,
               Manifold& mout,
               Jacobian& J_c_a, Jacobian& J_c_b) const;

  /// SO2 specific functions

  /*const*/ Scalar/*&*/ real() const;
  /*const*/ Scalar/*&*/ imag() const;
  Scalar angle() const;
  void angle(const Scalar theta);

protected:

  /// @todo given a Eigen::Map<const SO2>
  /// data()->x() return a reference to
  /// temporary ...

//  Scalar& real();
//  Scalar& imag();
};

template <typename _Derived>
typename SO2Base<_Derived>::Transformation
SO2Base<_Derived>::matrix() const
{
  return Transformation();
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
//  imag() = 1;
  data()->setIdentity();
}

template <typename _Derived>
void SO2Base<_Derived>::random()
{
  data()->setRandom();
}

template <typename _Derived>
typename SO2Base<_Derived>::Manifold
SO2Base<_Derived>::inverse() const
{
  return Manifold(real(), -imag());
}
/*
template <typename _Derived>
typename SO2Base<_Derived>::Manifold
SO2Base<_Derived>::rplus(const Tangent& t) const
{
  /// @todo check this
  return compose(t.retract());
}

template <typename _Derived>
typename SO2Base<_Derived>::Manifold
SO2Base<_Derived>::lplus(const Tangent& t) const
{
  // In SO2 rotation are commutative
  return rplus(t);
}

template <typename _Derived>
typename SO2Base<_Derived>::Manifold
SO2Base<_Derived>::rminus(const Manifold& m) const
{
  return Manifold(real() - m.real(),
                  imag() - m.imag());
}

template <typename _Derived>
typename SO2Base<_Derived>::Manifold
SO2Base<_Derived>::lminus(const Manifold& m) const
{
  return Manifold(m.real() - real(),
                  m.imag() - imag());
}
*/
template <typename _Derived>
typename SO2Base<_Derived>::Tangent
SO2Base<_Derived>::lift() const
{
  return Tangent(angle());
}

template <typename _Derived>
typename SO2Base<_Derived>::Manifold
SO2Base<_Derived>::compose(const Manifold& m) const
{
  const Scalar& lhs_real = real();
  const Scalar& lhs_imag = imag();
  const Scalar& rhs_real = m.real();
  const Scalar& rhs_imag = m.imag();

  return Manifold(
        lhs_real * rhs_real - lhs_imag * rhs_imag,
        lhs_real * rhs_imag + lhs_imag * rhs_real
        );
}

/// with Jacs

template <typename _Derived>
void SO2Base<_Derived>::inverse(Manifold& m, Jacobian& j) const
{
  m = inverse();
  j = Jacobian(1);
}

template <typename _Derived>
void SO2Base<_Derived>::rplus(const Tangent& t,
                              Manifold& m,
                              Jacobian& J_c_a,
                              Jacobian& J_c_b) const
{
  m = rplus(t);
  J_c_a.setIdentity();
  J_c_b = rotation();
}

template <typename _Derived>
void SO2Base<_Derived>::lplus(const Tangent& t,
                              Manifold& m,
                              Jacobian& J_c_a,
                              Jacobian& J_c_b) const
{
  m = lplus(t);
  J_c_a = t.retract().rotation();
  J_c_b.setIdentity();
}

template <typename _Derived>
void SO2Base<_Derived>::rminus(const Manifold& min,
                               Manifold& mout,
                               Jacobian& J_c_a,
                               Jacobian& J_c_b) const
{
  mout = rminus(min);
  J_c_a = -rotation().transpose();
  J_c_b =  rotation().transpose();
}

template <typename _Derived>
void SO2Base<_Derived>::lminus(const Manifold& min,
                               Manifold& mout,
                               Jacobian& J_c_a,
                               Jacobian& J_c_b) const
{
  mout = lminus(min);
  J_c_a =  min.rotation().transpose();
  J_c_b = -min.rotation().transpose();
}

template <typename _Derived>
void SO2Base<_Derived>::lift(const Manifold& m,
                             Tangent& t,
                             Jacobian& J_t_m) const
{
  t = m.lift();
//  J_t_m = ;
}

template <typename _Derived>
void SO2Base<_Derived>::compose(const Manifold& ma,
                                const Manifold& mb,
                                Manifold& mout,
                                Jacobian& J_c_a,
                                Jacobian& J_c_b) const
{
  mout = ma.compose(mb);
  J_c_a.setIdentity();
  J_c_b = rotation();
}

/// SO2 specific function

template <typename _Derived>
/*const*/ typename SO2Base<_Derived>::Scalar/*&*/
SO2Base<_Derived>::real() const
{
  return data()->x();
}

template <typename _Derived>
/*const*/ typename SO2Base<_Derived>::Scalar/*&*/
SO2Base<_Derived>::imag() const
{
  return data()->y();
}

template <typename _Derived>
typename SO2Base<_Derived>::Scalar
SO2Base<_Derived>::angle() const
{
  using std::atan2;
  return atan2(imag(), real());
}

template <typename _Derived>
void SO2Base<_Derived>::angle(const Scalar theta)
{
  using std::cos;
  using std::sin;

  data()->operator()(0) = cos(theta);
  data()->operator()(1) = sin(theta);
}

//template <typename _Derived>
//typename SO2Base<_Derived>::Scalar&
//SO2Base<_Derived>::real()
//{
//  return data()->x();
//}

//template <typename _Derived>
//typename SO2Base<_Derived>::Scalar&
//SO2Base<_Derived>::imag()
//{
//  return data()->y();
//}

} /* namespace manif */

#endif /* _MANIF_MANIF_SO2_BASE_H_ */
