#ifndef _MANIF_MANIF_MANIFOLD_BASE_H_
#define _MANIF_MANIF_MANIFOLD_BASE_H_

#include "manif/impl/macro.h"
#include "manif/impl/traits.h"
#include "manif/constants.h"
#include "manif/impl/tangent_base.h"

#include "optional.hpp"
#include "lspdlog/logging.h"

namespace manif
{

template <class _Derived>
struct ManifoldBase
{
  static constexpr int Dim     = internal::traits<_Derived>::Dim;
  static constexpr int DoF     = internal::traits<_Derived>::DoF;
  static constexpr int N       = internal::traits<_Derived>::N;
  static constexpr int RepSize = internal::traits<_Derived>::RepSize;

  using Scalar         = typename internal::traits<_Derived>::Scalar;
  using Manifold       = typename internal::traits<_Derived>::Manifold;
  using DataType       = typename internal::traits<_Derived>::DataType;
  using Tangent        = typename internal::traits<_Derived>::Tangent;
  using Jacobian       = typename internal::traits<_Derived>::Jacobian;
  using Transformation = typename internal::traits<_Derived>::Transformation;
  using Rotation       = typename internal::traits<_Derived>::Rotation;
  using Vector         = typename internal::traits<_Derived>::Vector;

  using OptJacobianRef = tl::optional<Jacobian&>;

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

  static const OptJacobianRef _;

  const DataType& coeffs() const;

  Scalar* data();
  const Scalar* data() const;

  template <class _NewScalar>
  ManifoldTemplate<_NewScalar> cast() const;

  Transformation transform() const;

  Rotation rotation() const;

  void identity();
  void random();

  // Minimum API

  Manifold inverse(OptJacobianRef J_m_t = {}) const;

  Tangent lift(OptJacobianRef J_t_m = {}) const;

  template <typename _DerivedOther>
  Manifold compose(const ManifoldBase<_DerivedOther>& m,
                   OptJacobianRef J_mc_ma = {},
                   OptJacobianRef J_mc_mb = {}) const;

  Vector act(const Vector& v,
             OptJacobianRef J_vout_m = {},
             OptJacobianRef J_vout_v = {}) const;

  // Deduced API

  template <typename _DerivedOther>
  Manifold rplus(const TangentBase<_DerivedOther>& t,
                 OptJacobianRef J_mout_m = {},
                 OptJacobianRef J_mout_t = {}) const;

  template <typename _DerivedOther>
  Manifold lplus(const TangentBase<_DerivedOther>& t,
                 OptJacobianRef J_mout_m = {},
                 OptJacobianRef J_mout_t = {}) const;

  /**
   * @brief plus, calls rplus
   * @see rplus
   */
  template <typename _DerivedOther>
  Manifold plus(const TangentBase<_DerivedOther>& t,
                OptJacobianRef J_mout_m = {},
                OptJacobianRef J_mout_t = {}) const;

  template <typename _DerivedOther>
  Tangent rminus(const ManifoldBase<_DerivedOther>& m,
                 OptJacobianRef J_t_ma = {},
                 OptJacobianRef J_t_mb = {}) const;

  template <typename _DerivedOther>
  Tangent lminus(const ManifoldBase<_DerivedOther>& m,
                 OptJacobianRef J_t_ma = {},
                 OptJacobianRef J_t_mb = {}) const;

  /**
   * @brief minus, calls rminus
   * @see rminus
   */
  template <typename _DerivedOther>
  Tangent minus(const ManifoldBase<_DerivedOther>& m,
                OptJacobianRef J_t_ma = {},
                OptJacobianRef J_t_mb = {}) const;

  template <typename _DerivedOther>
  Manifold between(const ManifoldBase<_DerivedOther>& m,
                   OptJacobianRef J_mc_ma = {},
                   OptJacobianRef J_mc_mb = {}) const;

  /// @todo
//  LieType lie() const {return derived().lie();}
//  Manifold interpolate() {return derived().interpolate();}

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

  /// Some static helpers

  static Manifold Identity();
  static Manifold Random();

private:

  _Derived& derived() { return *static_cast< _Derived* >(this); }
  const _Derived& derived() const { return *static_cast< const _Derived* >(this); }
};

template <typename _Derived>
const typename ManifoldBase<_Derived>::OptJacobianRef
ManifoldBase<_Derived>::_ = {};

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
template <class _NewScalar>
typename ManifoldBase<_Derived>::template ManifoldTemplate<_NewScalar>
ManifoldBase<_Derived>::cast() const
{
  return ManifoldTemplate<_NewScalar>(coeffs().cast<_NewScalar>());
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
  coeffs_nonconst() = Tangent::Random().retract().coeffs();
}

template <typename _Derived>
typename ManifoldBase<_Derived>::Manifold
ManifoldBase<_Derived>::inverse(OptJacobianRef J_m_t) const
{
  return derived().inverse(J_m_t);
}

template <typename _Derived>
template <typename _DerivedOther>
typename ManifoldBase<_Derived>::Manifold
ManifoldBase<_Derived>::rplus(
    const TangentBase<_DerivedOther>& t,
    OptJacobianRef J_mout_m,
    OptJacobianRef J_mout_t) const
{
  Manifold rplus;

  if (J_mout_t)
  {
    Jacobian J_ret_t;
    Jacobian J_rplus_ret;
    rplus = compose(t.retract(J_ret_t), J_mout_m, J_rplus_ret);
    J_mout_t->noalias() = J_rplus_ret * J_ret_t;
  }
  else
  {
    rplus = compose(t.retract(), J_mout_m, _);
  }

  return rplus;
}

template <typename _Derived>
template <typename _DerivedOther>
typename ManifoldBase<_Derived>::Manifold
ManifoldBase<_Derived>::lplus(
    const TangentBase<_DerivedOther>& t,
    OptJacobianRef J_mout_m,
    OptJacobianRef J_mout_t) const
{
  Manifold lplus;

  if (J_mout_t)
  {
    Jacobian J_ret_t;
    Jacobian J_lplus_ret;

    lplus = t.retract(J_ret_t).compose(*this, J_lplus_ret, J_mout_m);

    (*J_mout_t) = J_lplus_ret * J_ret_t;
  }
  else
  {
    lplus = t.retract().compose(*this, _, J_mout_m);
  }

  return lplus;
}

template <typename _Derived>
template <typename _DerivedOther>
typename ManifoldBase<_Derived>::Manifold
ManifoldBase<_Derived>::plus(
    const TangentBase<_DerivedOther>& t,
    OptJacobianRef J_mout_m,
    OptJacobianRef J_mout_t) const
{
  return derived().rplus(t, J_mout_m, J_mout_t);
}

template <typename _Derived>
template <typename _DerivedOther>
typename ManifoldBase<_Derived>::Tangent
ManifoldBase<_Derived>::rminus(
    const ManifoldBase<_DerivedOther>& m,
    OptJacobianRef J_t_ma,
    OptJacobianRef J_t_mb) const
{
  Tangent t;

  /// @todo optimize this
  if (J_t_ma && J_t_mb)
  {
    Jacobian J_inv_mb;
    Jacobian J_comp_inv;
    Jacobian J_comp_ma;
    Jacobian J_rminus_comp;

    t = m.inverse(J_inv_mb).
          compose(derived(), J_comp_inv, J_comp_ma).
            lift(J_rminus_comp);

    (*J_t_ma) = J_rminus_comp * J_comp_ma;
    (*J_t_mb) = J_rminus_comp * J_comp_inv * J_inv_mb;
  }
  else if (J_t_ma && !J_t_mb)
  {
    Jacobian J_comp_ma;
    Jacobian J_rminus_comp;

    t = m.inverse().
          compose(derived(), _, J_comp_ma).
            lift(J_rminus_comp);

    (*J_t_ma) = J_rminus_comp * J_comp_ma;
  }
  else if (!J_t_ma && J_t_mb)
  {
    Jacobian J_inv_mb;
    Jacobian J_comp_inv;
    Jacobian J_rminus_comp;

    t = m.inverse(J_inv_mb).
          compose(derived(), J_comp_inv, _).
            lift(J_rminus_comp);

    (*J_t_mb) = J_rminus_comp * J_comp_inv * J_inv_mb;
  }
  else
  {
    t = m.inverse().compose(derived()).lift();
  }

  return t;
}

template <typename _Derived>
template <typename _DerivedOther>
typename ManifoldBase<_Derived>::Tangent
ManifoldBase<_Derived>::lminus(
    const ManifoldBase<_DerivedOther>& m,
    OptJacobianRef J_t_ma,
    OptJacobianRef J_t_mb) const
{
  Tangent t;

  /// @todo optimize this
  if (J_t_ma && J_t_mb)
  {
    Jacobian J_inv_ma;
    Jacobian J_comp_inv;
    Jacobian J_comp_mb;
    Jacobian J_rminus_comp;

    t = m.compose(derived().inverse(J_inv_ma),
                  J_comp_mb, J_comp_inv).lift(J_rminus_comp);

    (*J_t_ma) = J_rminus_comp * J_comp_inv * J_inv_ma;
    (*J_t_mb) = J_rminus_comp * J_comp_mb;
  }
  else if (J_t_ma && !J_t_mb)
  {
    Jacobian J_inv_ma;
    Jacobian J_comp_inv;
    Jacobian J_rminus_comp;

    t = m.compose(derived().inverse(J_inv_ma),
                  _, J_comp_inv).lift(J_rminus_comp);

    (*J_t_ma) = J_rminus_comp * J_comp_inv * J_inv_ma;
  }
  else if (!J_t_ma && J_t_mb)
  {
    Jacobian J_comp_mb;
    Jacobian J_rminus_comp;

    t = m.compose(derived().inverse(),
                  J_comp_mb, _).lift(J_rminus_comp);

    (*J_t_mb) = J_rminus_comp * J_comp_mb;
  }
  else
  {
    t = derived().inverse().compose(m).lift();
  }

  return t;
}

template <typename _Derived>
template <typename _DerivedOther>
typename ManifoldBase<_Derived>::Tangent
ManifoldBase<_Derived>::minus(
    const ManifoldBase<_DerivedOther>& m,
    OptJacobianRef J_t_ma,
    OptJacobianRef J_t_mb) const
{
  return derived().rminus(m, J_t_ma, J_t_mb);
}

template <typename _Derived>
typename ManifoldBase<_Derived>::Tangent
ManifoldBase<_Derived>::lift(OptJacobianRef J_t_m) const
{
  return derived().lift(J_t_m);
}

template <typename _Derived>
template <typename _DerivedOther>
typename ManifoldBase<_Derived>::Manifold
ManifoldBase<_Derived>::compose(
    const ManifoldBase<_DerivedOther>& m,
    OptJacobianRef J_mc_ma,
    OptJacobianRef J_mc_mb) const
{
  return derived().compose(m, J_mc_ma, J_mc_mb);
}

template <typename _Derived>
template <typename _DerivedOther>
typename ManifoldBase<_Derived>::Manifold
ManifoldBase<_Derived>::between(
    const ManifoldBase<_DerivedOther>& m,
    OptJacobianRef J_mc_ma,
    OptJacobianRef J_mc_mb) const
{
  Manifold between;

  if (J_mc_ma)
  {
    Jacobian J_inv_ma;
    Jacobian J_mc_inv;
    between = derived().inverse(J_inv_ma).compose(m, J_mc_inv, J_mc_mb);

    (*J_mc_ma) = J_mc_inv * J_inv_ma;
  }
  else
  {
    between = derived().inverse().compose(m, _, J_mc_mb);
  }


  return between;
}

template <typename _Derived>
typename ManifoldBase<_Derived>::Vector
ManifoldBase<_Derived>::act(const Vector& v,
                            OptJacobianRef J_vout_m,
                            OptJacobianRef J_vout_v) const
{
  return derived().act(v, J_vout_m, J_vout_v);
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

/// Static helpers

template <typename _Derived>
typename ManifoldBase<_Derived>::Manifold
ManifoldBase<_Derived>::Identity()
{
  /// @todo how to optimize .identity() call away ?
  static Manifold m; m.identity();
  return m;
}

template <typename _Derived>
typename ManifoldBase<_Derived>::Manifold
ManifoldBase<_Derived>::Random()
{
  /// @todo how to optimize .random() call away ?
  static Manifold m; m.random();
  return m;
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
