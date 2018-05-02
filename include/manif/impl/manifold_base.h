#ifndef _MANIF_MANIF_MANIFOLD_BASE_H_
#define _MANIF_MANIF_MANIFOLD_BASE_H_

#include "manif/impl/macro.h"
#include "manif/impl/traits.h"
#include "manif/constants.h"
#include "manif/impl/tangent_base.h"

#include "lspdlog/logging.h"

namespace manif
{

template <class _Derived>
struct ManifoldBase;

template <typename _Manifold>
struct traits_cast;

template <>
template <template <typename _Scalar> class _Manifold, typename _Scalar>
struct traits_cast<_Manifold<_Scalar>>
{
  template <typename T>
  using MaMan = _Manifold<T>;

  using MyInt = int;
};

template <class _Derived>
struct ManifoldBase
{
  static constexpr int Dim     = internal::traits<_Derived>::Dim;
  static constexpr int DoF     = internal::traits<_Derived>::DoF;
  static constexpr int N       = internal::traits<_Derived>::N;
  static constexpr int RepSize = internal::traits<_Derived>::RepSize;

  using Scalar   = typename internal::traits<_Derived>::Scalar;
  using Manifold = typename internal::traits<_Derived>::Manifold;
  using DataType       = typename internal::traits<_Derived>::DataType;
  using Tangent        = typename internal::traits<_Derived>::Tangent;
  using Jacobian       = typename internal::traits<_Derived>::Jacobian;
  using Transformation = typename internal::traits<_Derived>::Transformation;
  using Rotation       = typename internal::traits<_Derived>::Rotation;
  using Vector         = typename internal::traits<_Derived>::Vector;

  //  using MyInt = typename traits_cast<_Derived>::MyInt;

  /// @todo find something sexier
  template <typename T>
  using ManifoldTemplate =
  typename internal::traits<_Derived>::template ManifoldTemplate<T>;

  /// @todo this is an implicit conversion operator,
  /// evaluate how bad it is to use it.
  operator _Derived&() { return derived(); }
  operator const _Derived& () const { return derived(); }

protected:

  DataType& coeffs_nonconst();

public:

  const DataType& coeffs() const;

  Scalar* data();
  const Scalar* data() const;

  /// @todo
  template <class _NewScalar>
  ManifoldTemplate<_NewScalar> cast() const
  {
    return ManifoldTemplate<_NewScalar>(coeffs().cast<_NewScalar>());
  }

  Transformation transform() const;

  Rotation rotation() const;

  void identity();

  void random();

  Manifold inverse() const;

  template <typename _DerivedOther>
  Manifold rplus(const TangentBase<_DerivedOther>& t) const;
  template <typename _DerivedOther>
  Manifold lplus(const TangentBase<_DerivedOther>& t) const;

  /**
   * @brief plus, calls rplus
   * @see rplus
   */
  template <typename _DerivedOther>
  Manifold plus(const TangentBase<_DerivedOther>& t) const;

  template <typename _DerivedOther>
  Tangent rminus(const ManifoldBase<_DerivedOther>& m) const;
  template <typename _DerivedOther>
  Tangent lminus(const ManifoldBase<_DerivedOther>& m) const;

  /**
   * @brief minus, calls rminus
   * @see rminus
   */
  template <typename _DerivedOther>
  Tangent minus(const ManifoldBase<_DerivedOther>& m) const;

  Tangent lift() const;

  template <typename _DerivedOther>
  Manifold compose(const ManifoldBase<_DerivedOther>& m) const;

  template <typename _DerivedOther>
  Manifold between(const ManifoldBase<_DerivedOther>& m) const;

  Vector act(const Vector& v) const;

  /// @todo
//  LieType lie() const {return derived().lie();}
//  Manifold interpolate() {return derived().interpolate();}
//  Vector act(const Vector& v) {return derived().act(v);}

  /// Some operators

  /**
   * @brief operator =, assignment oprator
   * @param t
   * @return
   */
  _Derived& operator =(const ManifoldBase<_Derived>& m);

  template <typename _DerivedOther>
  _Derived& operator =(const ManifoldBase<_DerivedOther>& m);

  /**
   * @brief operator +, rplus
   * @see rplus
   */
  template <typename _DerivedOther>
  Manifold operator +(const TangentBase<_DerivedOther>& t) const;

  /**
   * @brief operator +=, in-place rplus
   * @see rplus
   */
  template <typename _DerivedOther>
  _Derived& operator +=(const TangentBase<_DerivedOther>& t);

  /**
   * @brief operator -, rminus
   * @see rminus
   */
  template <typename _DerivedOther>
  Tangent operator -(const ManifoldBase<_DerivedOther>& m) const;

  /**
   * @brief operator *, compose
   * @see compose
   */
  template <typename _DerivedOther>
  Manifold operator *(const ManifoldBase<_DerivedOther>& m) const;

  /**
   * @brief operator *=, in-place compose
   * @see compose
   */
  template <typename _DerivedOther>
  _Derived& operator *=(const ManifoldBase<_DerivedOther>& m);

  /// Jacs

  template <typename _DerivedOther>
  void inverse(ManifoldBase<_DerivedOther>& m, Jacobian& J) const;

  template <typename _DerivedOther>
  void lift(ManifoldBase<_DerivedOther>& m, Jacobian& J) const;

  template <typename _DerivedOther0, typename _DerivedOther1>
  void compose(const ManifoldBase<_DerivedOther0>& mb,
               ManifoldBase<_DerivedOther1>& mc,
               Jacobian& J_mc_ma, Jacobian& J_mc_mb) const;

  template <typename _DerivedOther0, typename _DerivedOther1>
  void rplus(const TangentBase<_DerivedOther0>& t,
             ManifoldBase<_DerivedOther1>& m,
             Jacobian& J_mout_m, Jacobian& J_mout_t) const;

  template <typename _DerivedOther0, typename _DerivedOther1>
  void lplus(const TangentBase<_DerivedOther0>& t,
             ManifoldBase<_DerivedOther1>& m,
             Jacobian& J_mout_m, Jacobian& J_mout_t) const;

  template <typename _DerivedOther0, typename _DerivedOther1>
  void plus(const TangentBase<_DerivedOther0>& t,
            ManifoldBase<_DerivedOther1>& m,
            Jacobian& J_mout_m, Jacobian& J_mout_t) const;

  template <typename _DerivedOther0, typename _DerivedOther1>
  void rminus(const ManifoldBase<_DerivedOther0>& mb,
              TangentBase<_DerivedOther1>& t,
              Jacobian& J_t_ma, Jacobian& J_t_mb) const;

  template <typename _DerivedOther0, typename _DerivedOther1>
  void lminus(const ManifoldBase<_DerivedOther0>& mb,
              TangentBase<_DerivedOther1>& t,
              Jacobian& J_t_ma, Jacobian& J_t_mb) const;

  template <typename _DerivedOther0, typename _DerivedOther1>
  void minus(const ManifoldBase<_DerivedOther0>& mb,
             TangentBase<_DerivedOther1>& t,
             Jacobian& J_t_ma, Jacobian& J_t_mb) const;

  template <typename _DerivedOther0, typename _DerivedOther1>
  void between(const ManifoldBase<_DerivedOther0>& mb,
               ManifoldBase<_DerivedOther1>& mc,
               Jacobian& J_mc_ma, Jacobian& J_mc_mb) const;

  /// Some static helpers

  static Manifold Identity()
  {
    /// @todo how to optimize .identity() call away ?
    static Manifold m; m.identity();
    return m;
  }

  static Manifold Random()
  {
    /// @todo how to optimize .random() call away ?
    static Manifold m; m.random();
    return m;
  }

  static Tangent Inverse(const Manifold& m);

  static Manifold Rplus(const Manifold& m, const Tangent& t);

  static Manifold Lplus(const Tangent& t, const Manifold& m);

  static Manifold Rminus(const Manifold& m0, const Manifold& m1);

  static Manifold Lminus(const Manifold& m0, const Manifold& m1);

  static Tangent Lift(const Manifold& m);

  static Manifold Retract(const Tangent& t);

  /*
  static LieType Lie(const Manifold& m)
  static LieType Lie(const Tangent& t)
  */

  static Manifold Compose(const Manifold& m0, const Manifold& m1);
  static Manifold Between(const Manifold& m0, const Manifold& m1);

  /// static helpers with Jacobians

  template <typename _Jacobian>
  static void Inverse(const Manifold& m, Manifold& minv, _Jacobian& jac);

private:

  _Derived& derived() { return *static_cast< _Derived* >(this); }
  const _Derived& derived() const { return *static_cast< const _Derived* >(this); }
};

template <typename _Derived>
typename ManifoldBase<_Derived>::DataType&
ManifoldBase<_Derived>::coeffs_nonconst()
{
  return derived().coeffs_nonconst();
}

template <typename _Derived>
const typename ManifoldBase<_Derived>::DataType&
ManifoldBase<_Derived>::coeffs() const
{
  return derived().coeffs();
}

template <typename _Derived>
typename ManifoldBase<_Derived>::Scalar*
ManifoldBase<_Derived>::data()
{
  return derived().coeffs_nonconst().data();
}

template <typename _Derived>
const typename ManifoldBase<_Derived>::Scalar*
ManifoldBase<_Derived>::data() const
{
  derived().coeffs().data();
}

template <typename _Derived>
typename ManifoldBase<_Derived>::Transformation
ManifoldBase<_Derived>::transform() const
{
  return derived().transform();
}

template <typename _Derived>
typename ManifoldBase<_Derived>::Rotation
ManifoldBase<_Derived>::rotation() const
{
  return derived().rotation();
}

template <typename _Derived>
void ManifoldBase<_Derived>::identity()
{
  derived().identity();
}

template <typename _Derived>
void ManifoldBase<_Derived>::random()
{
//  const auto m = Tangent::Random().retract();
//  coeffs_nonconst() = m.coeffs_nonconst();

  coeffs_nonconst() = Tangent::Random().retract().coeffs();
}

template <typename _Derived>
typename ManifoldBase<_Derived>::Manifold
ManifoldBase<_Derived>::inverse() const
{
  return derived().inverse();
}

template <typename _Derived>
template <typename _DerivedOther>
typename ManifoldBase<_Derived>::Manifold
ManifoldBase<_Derived>::rplus(
    const TangentBase<_DerivedOther>& t) const
{
  return compose(t.retract());
}

template <typename _Derived>
template <typename _DerivedOther>
typename ManifoldBase<_Derived>::Manifold
ManifoldBase<_Derived>::lplus(
    const TangentBase<_DerivedOther>& t) const
{
  return t.retract().compose(derived());
}

template <typename _Derived>
template <typename _DerivedOther>
typename ManifoldBase<_Derived>::Manifold
ManifoldBase<_Derived>::plus(
    const TangentBase<_DerivedOther>& t) const
{
  return rplus(t);
}

template <typename _Derived>
template <typename _DerivedOther>
typename ManifoldBase<_Derived>::Tangent
ManifoldBase<_Derived>::rminus(
    const ManifoldBase<_DerivedOther>& m) const
{
  return m.inverse().compose(derived()).lift();
}

template <typename _Derived>
template <typename _DerivedOther>
typename ManifoldBase<_Derived>::Tangent
ManifoldBase<_Derived>::lminus(
    const ManifoldBase<_DerivedOther>& m) const
{
  return derived().inverse().compose(m).lift();
}

template <typename _Derived>
template <typename _DerivedOther>
typename ManifoldBase<_Derived>::Tangent
ManifoldBase<_Derived>::minus(
    const ManifoldBase<_DerivedOther>& m) const
{
  return rminus(m);
}

template <typename _Derived>
typename ManifoldBase<_Derived>::Tangent
ManifoldBase<_Derived>::lift() const
{
  return derived().lift();
}

template <typename _Derived>
template <typename _DerivedOther>
typename ManifoldBase<_Derived>::Manifold
ManifoldBase<_Derived>::compose(
    const ManifoldBase<_DerivedOther>& m) const
{
  return derived().compose(m);
}

template <typename _Derived>
template <typename _DerivedOther>
typename ManifoldBase<_Derived>::Manifold
ManifoldBase<_Derived>::between(
    const ManifoldBase<_DerivedOther>& m) const
{
  return derived().inverse().compose(m);
}

template <typename _Derived>
typename ManifoldBase<_Derived>::Vector
ManifoldBase<_Derived>::act(const Vector& v) const
{
  return v.transpose() * transform();
}

/// Operators

template <typename _Derived>
_Derived&
ManifoldBase<_Derived>::operator =(
    const ManifoldBase<_Derived>& m)
{
  derived().coeffs_nonconst() = m.coeffs();
  return derived();
}

template <typename _Derived>
template <typename _DerivedOther>
_Derived&
ManifoldBase<_Derived>::operator =(
    const ManifoldBase<_DerivedOther>& m)
{
  derived().coeffs_nonconst() = m.coeffs();
  return derived();
}

template <typename _Derived>
template <typename _DerivedOther>
typename ManifoldBase<_Derived>::Manifold
ManifoldBase<_Derived>::operator +(
    const TangentBase<_DerivedOther>& t) const
{
  return derived().rplus(t);
}

template <typename _Derived>
template <typename _DerivedOther>
_Derived&
ManifoldBase<_Derived>::operator +=(
    const TangentBase<_DerivedOther>& t)
{
  derived() = derived().rplus(t);
  return derived();
}

template <typename _Derived>
template <typename _DerivedOther>
typename ManifoldBase<_Derived>::Tangent
ManifoldBase<_Derived>::operator -(
    const ManifoldBase<_DerivedOther>& m) const
{
  return derived().rminus(m);
}

template <typename _Derived>
template <typename _DerivedOther>
typename ManifoldBase<_Derived>::Manifold
ManifoldBase<_Derived>::operator *(
    const ManifoldBase<_DerivedOther>& m) const
{
  return derived().compose(m);
}

template <typename _Derived>
template <typename _DerivedOther>
_Derived&
ManifoldBase<_Derived>::operator *=(
    const ManifoldBase<_DerivedOther>& m)
{
  derived() = derived().compose(m);
  return derived();
}

/// Jacs

template <typename _Derived>
template <typename _DerivedOther>
void ManifoldBase<_Derived>::inverse(
    ManifoldBase<_DerivedOther>& m, Jacobian& J) const
{
  derived().inverse(m, J);
}

template <typename _Derived>
template <typename _DerivedOther>
void ManifoldBase<_Derived>::lift(
    ManifoldBase<_DerivedOther>& m, Jacobian& J) const
{
  derived().lift(m, J);
}

template <typename _Derived>
template <typename _DerivedOther0, typename _DerivedOther1>
void ManifoldBase<_Derived>::compose(
    const ManifoldBase<_DerivedOther0>& mb,
    ManifoldBase<_DerivedOther1>& mc,
    Jacobian& J_mc_ma,
    Jacobian& J_mc_mb) const
{
  derived().compose(mb, mc, J_mc_ma, J_mc_mb);
}

template <typename _Derived>
template <typename _DerivedOther0, typename _DerivedOther1>
void ManifoldBase<_Derived>::rplus(
    const TangentBase<_DerivedOther0>& t,
    ManifoldBase<_DerivedOther1>& m,
    Jacobian& J_rplus_m,
    Jacobian& J_rplus_t) const
{
  Manifold ret;
  Jacobian J_ret_t;

  t.retract(ret, J_ret_t);

  Jacobian J_rplus_ret;

  compose(ret, m, J_rplus_m, J_rplus_ret);

  J_rplus_t = J_rplus_ret * J_ret_t;
}

template <typename _Derived>
template <typename _DerivedOther0, typename _DerivedOther1>
void ManifoldBase<_Derived>::lplus(
    const TangentBase<_DerivedOther0>& t,
    ManifoldBase<_DerivedOther1>& m,
    Jacobian& J_lplus_m,
    Jacobian& J_lplus_t) const
{
  Manifold ret;
  Jacobian J_ret_t;

  t.retract(ret, J_ret_t);

  Jacobian J_lplus_ret;

  ret.compose(*this, m, J_lplus_ret, J_lplus_m);

  J_lplus_t = J_lplus_ret * J_ret_t;
}

template <typename _Derived>
template <typename _DerivedOther0, typename _DerivedOther1>
void ManifoldBase<_Derived>::plus(
    const TangentBase<_DerivedOther0>& t,
    ManifoldBase<_DerivedOther1>& m,
    Jacobian& J_mout_m,
    Jacobian& J_mout_t) const
{
  rplus(t, m, J_mout_m, J_mout_t);
}

template <typename _Derived>
template <typename _DerivedOther0, typename _DerivedOther1>
void ManifoldBase<_Derived>::rminus(
    const ManifoldBase<_DerivedOther0>& mb,
    TangentBase<_DerivedOther1>& t,
    Jacobian& J_rminus_ma,
    Jacobian& J_rminus_mb) const
{
  Manifold inv;
  Jacobian J_inv_mb;
  mb.inverse(inv, J_inv_mb);

  Manifold comp;
  Jacobian J_comp_inv;
  Jacobian J_comp_ma;
  inv.compose(*this, comp, J_comp_inv, J_comp_ma);

  Jacobian J_rminus_comp;
  comp.lift(t, J_rminus_comp);

  J_rminus_ma = J_rminus_comp * J_comp_ma;
  J_rminus_mb = J_rminus_comp * J_comp_inv * J_inv_mb;
}

template <typename _Derived>
template <typename _DerivedOther0, typename _DerivedOther1>
void ManifoldBase<_Derived>::lminus(
    const ManifoldBase<_DerivedOther0>& mb,
    TangentBase<_DerivedOther1>& t,
    Jacobian& J_lminus_ma,
    Jacobian& J_lminus_mb) const
{
  Manifold inv;
  Jacobian J_inv_ma;
  derived().inverse(inv, J_inv_ma);

  Manifold comp;
  Jacobian J_comp_inv;
  Jacobian J_comp_mb;
  mb.compose(inv, comp, J_comp_mb, J_comp_inv);

  Jacobian J_rminus_comp;
  comp.lift(t, J_rminus_comp);

  J_lminus_ma = J_rminus_comp * J_comp_inv * J_inv_ma;
  J_lminus_mb = J_rminus_comp * J_comp_mb;
}

template <typename _Derived>
template <typename _DerivedOther0, typename _DerivedOther1>
void ManifoldBase<_Derived>::minus(
    const ManifoldBase<_DerivedOther0>& mb,
    TangentBase<_DerivedOther1>& t,
    Jacobian& J_t_ma,
    Jacobian& J_t_mb) const
{
  rminus(mb, t, J_t_ma, J_t_mb);
}

template <typename _Derived>
template <typename _DerivedOther0, typename _DerivedOther1>
void ManifoldBase<_Derived>::between(
    const ManifoldBase<_DerivedOther0>& mb,
    ManifoldBase<_DerivedOther1>& mc,
    Jacobian& J_mc_ma, Jacobian& J_mc_mb) const
{
  Manifold inv;
  Jacobian J_inv_ma;
  derived().inverse(inv, J_inv_ma);

  Jacobian J_mc_inv;
  inv.compose(mb, mc, J_mc_inv, J_mc_mb);

  J_mc_ma = J_mc_inv * J_inv_ma;
}

/// Utils

template <typename _Stream, typename _Derived>
_Stream& operator << (
    _Stream& s,
    const manif::ManifoldBase<_Derived>& m)
{
  s << m.coeffs().transpose();
}

} /* namespace manif */

#endif /* _MANIF_MANIF_MANIFOLD_BASE_H_ */
