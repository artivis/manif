#ifndef _MANIF_MANIF_LIE_GROUP_BASE_H_
#define _MANIF_MANIF_LIE_GROUP_BASE_H_

#include "manif/impl/macro.h"
#include "manif/impl/traits.h"
#include "manif/impl/eigen.h"
#include "manif/impl/tangent_base.h"

#include "manif/constants.h"

#include "lt/optional.hpp"

namespace manif {

template <class _Derived>
struct LieGroupBase
{
  static constexpr int Dim     = internal::traits<_Derived>::Dim;
  static constexpr int DoF     = internal::traits<_Derived>::DoF;
  static constexpr int N       = internal::traits<_Derived>::N;
  static constexpr int RepSize = internal::traits<_Derived>::RepSize;

  using Scalar         = typename internal::traits<_Derived>::Scalar;
  using LieGroup       = typename internal::traits<_Derived>::LieGroup;
  using DataType       = typename internal::traits<_Derived>::DataType;
  using Tangent        = typename internal::traits<_Derived>::Tangent;
  using Jacobian       = typename internal::traits<_Derived>::Jacobian;
  using Transformation = typename internal::traits<_Derived>::Transformation; /// @todo rename ActionMat?
  using Rotation       = typename internal::traits<_Derived>::Rotation;
  using Vector         = typename internal::traits<_Derived>::Vector;

  using OptJacobianRef = tl::optional<Eigen::Ref<Jacobian>>;

  template <typename _Scalar>
  using LieGroupTemplate = typename manif::internal::traitscast<LieGroup, _Scalar>::cast;

  /// @todo this is an implicit conversion operator,
  /// evaluate how bad it is to use it.
  operator _Derived&() { return derived(); }
  operator const _Derived&() const { return derived(); }

protected:

  //! @brief Access the underlying data by reference
  DataType& coeffs_nonconst();

public:

  static const OptJacobianRef _;

  //! @brief Access the underlying data by const reference
  const DataType& coeffs() const;

  //! @brief Access the underlying data by pointer
  Scalar* data();
  const Scalar* data() const;

  //! @brief Cast the LieGroup object to a copy object
  //! of a different scalar type
  template <class _NewScalar>
  LieGroupTemplate<_NewScalar> cast() const;

  /// @todo 'cast' across groups
  /// SO3 so3 = so2.as<SO3>()
//  template <class _DerivedOther>
//  LieGroupTemplate<_DerivedOther> as() const;

  //! @brief Set the current LieGroup object to Identity
  _Derived& setIdentity();

  //! @brief Set the current LieGroup object with random value
  _Derived& setRandom();

  // Minimum API
  // Those are the functions the Derived class must implement !

  //! @brief Return the inverse of the current LieGroup object
  LieGroup inverse(OptJacobianRef J_m_t = {}) const;

  //! @brief Return the Tangent of the current LieGroup object
  Tangent lift(OptJacobianRef J_t_m = {}) const;

  template <typename _DerivedOther>
  LieGroup compose(const LieGroupBase<_DerivedOther>& m,
                   OptJacobianRef J_mc_ma = {},
                   OptJacobianRef J_mc_mb = {}) const;

  Vector act(const Vector& v,
             OptJacobianRef J_vout_m = {},
             OptJacobianRef J_vout_v = {}) const;

  Jacobian adj() const;

  // Deduced API

  template <typename _DerivedOther>
  LieGroup rplus(const TangentBase<_DerivedOther>& t,
                 OptJacobianRef J_mout_m = {},
                 OptJacobianRef J_mout_t = {}) const;

  template <typename _DerivedOther>
  LieGroup lplus(const TangentBase<_DerivedOther>& t,
                 OptJacobianRef J_mout_m = {},
                 OptJacobianRef J_mout_t = {}) const;

  /**
   * @brief plus, calls rplus
   * @see rplus
   */
  template <typename _DerivedOther>
  LieGroup plus(const TangentBase<_DerivedOther>& t,
                OptJacobianRef J_mout_m = {},
                OptJacobianRef J_mout_t = {}) const;

  template <typename _DerivedOther>
  Tangent rminus(const LieGroupBase<_DerivedOther>& m,
                 OptJacobianRef J_t_ma = {},
                 OptJacobianRef J_t_mb = {}) const;

  template <typename _DerivedOther>
  Tangent lminus(const LieGroupBase<_DerivedOther>& m,
                 OptJacobianRef J_t_ma = {},
                 OptJacobianRef J_t_mb = {}) const;

  /**
   * @brief minus, calls rminus
   * @see rminus
   */
  template <typename _DerivedOther>
  Tangent minus(const LieGroupBase<_DerivedOther>& m,
                OptJacobianRef J_t_ma = {},
                OptJacobianRef J_t_mb = {}) const;

  template <typename _DerivedOther>
  LieGroup between(const LieGroupBase<_DerivedOther>& m,
                   OptJacobianRef J_mc_ma = {},
                   OptJacobianRef J_mc_mb = {}) const;

  template <typename _DerivedOther>
  bool isApprox(const LieGroupBase<_DerivedOther>& m, const Scalar eps) const;

  /// Some operators

  /**
   * @brief operator =, assignment oprator
   * @param t
   * @return
   */
  _Derived& operator =(const LieGroupBase<_Derived>& m);

  template <typename _DerivedOther>
  _Derived& operator =(const LieGroupBase<_DerivedOther>& m);

  template <typename _DerivedOther>
  bool operator ==(const LieGroupBase<_DerivedOther>& m);

  /**
   * @brief operator +, rplus
   * @see rplus
   */
  template <typename _DerivedOther>
  LieGroup operator +(const TangentBase<_DerivedOther>& t) const;

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
  Tangent operator -(const LieGroupBase<_DerivedOther>& m) const;

  /**
   * @brief operator *, compose
   * @see compose
   */
  template <typename _DerivedOther>
  LieGroup operator *(const LieGroupBase<_DerivedOther>& m) const;

  /**
   * @brief operator *=, in-place compose
   * @see compose
   */
  template <typename _DerivedOther>
  _Derived& operator *=(const LieGroupBase<_DerivedOther>& m);

  /// Some static helpers

  static LieGroup Identity();
  static LieGroup Random();

private:

  _Derived& derived() { return *static_cast< _Derived* >(this); }
  const _Derived& derived() const { return *static_cast< const _Derived* >(this); }
};

template <typename _Derived>
constexpr int LieGroupBase<_Derived>::Dim;
template <typename _Derived>
constexpr int LieGroupBase<_Derived>::DoF;
template <typename _Derived>
constexpr int LieGroupBase<_Derived>::N;
template <typename _Derived>
constexpr int LieGroupBase<_Derived>::RepSize;

template <typename _Derived>
const typename LieGroupBase<_Derived>::OptJacobianRef
LieGroupBase<_Derived>::_ = {};

template <typename _Derived>
typename LieGroupBase<_Derived>::DataType&
LieGroupBase<_Derived>::coeffs_nonconst()
{
  return derived().coeffs_nonconst();
}

template <typename _Derived>
const typename LieGroupBase<_Derived>::DataType&
LieGroupBase<_Derived>::coeffs() const
{
  return derived().coeffs();
}

template <typename _Derived>
typename LieGroupBase<_Derived>::Scalar*
LieGroupBase<_Derived>::data()
{
  return derived().coeffs_nonconst().data();
}

template <typename _Derived>
const typename LieGroupBase<_Derived>::Scalar*
LieGroupBase<_Derived>::data() const
{
  derived().coeffs().data();
}

template <typename _Derived>
template <class _NewScalar>
typename LieGroupBase<_Derived>::template LieGroupTemplate<_NewScalar>
LieGroupBase<_Derived>::cast() const
{
  return LieGroupTemplate<_NewScalar>(coeffs().template cast<_NewScalar>());
}

template <typename _Derived>
_Derived&
LieGroupBase<_Derived>::setIdentity()
{
  const static Tangent zero = Tangent::Zero();
  derived() = zero.retract();
  return derived();
}

template <typename _Derived>
_Derived&
LieGroupBase<_Derived>::setRandom()
{
  coeffs_nonconst() = Tangent::Random().retract().coeffs();
  return derived();
}

template <typename _Derived>
typename LieGroupBase<_Derived>::LieGroup
LieGroupBase<_Derived>::inverse(OptJacobianRef J_m_t) const
{
  return derived().inverse(J_m_t);
}

template <typename _Derived>
template <typename _DerivedOther>
typename LieGroupBase<_Derived>::LieGroup
LieGroupBase<_Derived>::rplus(
    const TangentBase<_DerivedOther>& t,
    OptJacobianRef J_mout_m,
    OptJacobianRef J_mout_t) const
{
  LieGroup mout;

  if (J_mout_t)
  {
    Jacobian J_ret_t;
    Jacobian J_mout_ret;
    mout = compose(t.retract(J_ret_t), J_mout_m, J_mout_ret);
    J_mout_t->noalias() = J_mout_ret * J_ret_t;
  }
  else
  {
    mout = compose(t.retract(), J_mout_m, _);
  }

  return mout;
}

template <typename _Derived>
template <typename _DerivedOther>
typename LieGroupBase<_Derived>::LieGroup
LieGroupBase<_Derived>::lplus(
    const TangentBase<_DerivedOther>& t,
    OptJacobianRef J_mout_m,
    OptJacobianRef J_mout_t) const
{
  LieGroup mout;

  if (J_mout_t)
  {
    Jacobian J_ret_t;
    Jacobian J_mout_ret;

    mout = t.retract(J_ret_t).compose(derived(), J_mout_ret, J_mout_m);

    J_mout_t->noalias() = J_mout_ret * J_ret_t;
  }
  else
  {
    mout = t.retract().compose(derived(), _, J_mout_m);
  }

  return mout;
}

template <typename _Derived>
template <typename _DerivedOther>
typename LieGroupBase<_Derived>::LieGroup
LieGroupBase<_Derived>::plus(
    const TangentBase<_DerivedOther>& t,
    OptJacobianRef J_mout_m,
    OptJacobianRef J_mout_t) const
{
  return derived().rplus(t, J_mout_m, J_mout_t);
}

template <typename _Derived>
template <typename _DerivedOther>
typename LieGroupBase<_Derived>::Tangent
LieGroupBase<_Derived>::rminus(
    const LieGroupBase<_DerivedOther>& m,
    OptJacobianRef J_t_ma,
    OptJacobianRef J_t_mb) const
{
  const Tangent t = m.inverse().compose(derived()).lift();

  if (J_t_ma)
  {
    (*J_t_ma) = t.rjacinv();
  }
  if (J_t_mb)
  {
    (*J_t_mb) = -(-t).rjacinv();
  }

  return t;
}

template <typename _Derived>
template <typename _DerivedOther>
typename LieGroupBase<_Derived>::Tangent
LieGroupBase<_Derived>::lminus(
    const LieGroupBase<_DerivedOther>& m,
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
    Jacobian J_t_comp;

    t = compose(m.inverse(J_inv_mb),
                J_comp_ma, J_comp_inv).lift(J_t_comp);

    J_t_ma->noalias() = J_t_comp * J_comp_ma;
    J_t_mb->noalias() = J_t_comp * J_comp_inv * J_inv_mb;
  }
  else if (J_t_ma && !J_t_mb)
  {
    Jacobian J_comp_a;
    Jacobian J_t_comp;

    t = compose(m.inverse(),
                J_comp_a, _ ).lift(J_t_comp);

    J_t_ma->noalias() = J_t_comp * J_comp_a;
  }
  else if (!J_t_ma && J_t_mb)
  {
    Jacobian J_inv_mb;
    Jacobian J_comp_inv;
    Jacobian J_t_comp;

    t = compose(m.inverse(J_inv_mb),
                _, J_comp_inv).lift(J_t_comp);

    J_t_mb->noalias() = J_t_comp * J_comp_inv * J_inv_mb;
  }
  else
  {
    t = compose(m.inverse()).lift();
  }

  return t;
}

template <typename _Derived>
template <typename _DerivedOther>
typename LieGroupBase<_Derived>::Tangent
LieGroupBase<_Derived>::minus(
    const LieGroupBase<_DerivedOther>& m,
    OptJacobianRef J_t_ma,
    OptJacobianRef J_t_mb) const
{
  return derived().rminus(m, J_t_ma, J_t_mb);
}

template <typename _Derived>
typename LieGroupBase<_Derived>::Tangent
LieGroupBase<_Derived>::lift(OptJacobianRef J_t_m) const
{
  return derived().lift(J_t_m);
}

template <typename _Derived>
template <typename _DerivedOther>
typename LieGroupBase<_Derived>::LieGroup
LieGroupBase<_Derived>::compose(
    const LieGroupBase<_DerivedOther>& m,
    OptJacobianRef J_mc_ma,
    OptJacobianRef J_mc_mb) const
{
  return derived().compose(m, J_mc_ma, J_mc_mb);
}

template <typename _Derived>
template <typename _DerivedOther>
typename LieGroupBase<_Derived>::LieGroup
LieGroupBase<_Derived>::between(
    const LieGroupBase<_DerivedOther>& m,
    OptJacobianRef J_mc_ma,
    OptJacobianRef J_mc_mb) const
{
  LieGroup mc;

  if (J_mc_ma)
  {
    Jacobian J_inv_ma;
    Jacobian J_mc_inv;
    mc = inverse(J_inv_ma).compose(m, J_mc_inv, J_mc_mb);

    J_mc_ma->noalias() = J_mc_inv * J_inv_ma;
  }
  else
  {
    mc = inverse().compose(m, _, J_mc_mb);
  }

  return mc;
}

template <typename _Derived>
template <typename _DerivedOther>
bool LieGroupBase<_Derived>::isApprox(const LieGroupBase<_DerivedOther>& m,
                                      const Scalar eps) const
{
  return rminus(m).isApprox(Tangent::Zero(), eps);
}

template <typename _Derived>
typename LieGroupBase<_Derived>::Jacobian
LieGroupBase<_Derived>::adj() const
{
  return derived().adj();
}

template <typename _Derived>
typename LieGroupBase<_Derived>::Vector
LieGroupBase<_Derived>::act(const Vector& v,
                            OptJacobianRef J_vout_m,
                            OptJacobianRef J_vout_v) const
{
  return derived().act(v, J_vout_m, J_vout_v);
}

/// Operators

template <typename _Derived>
_Derived&
LieGroupBase<_Derived>::operator =(
    const LieGroupBase<_Derived>& m)
{
  derived().coeffs_nonconst() = m.coeffs();
  return derived();
}

template <typename _Derived>
template <typename _DerivedOther>
_Derived&
LieGroupBase<_Derived>::operator =(
    const LieGroupBase<_DerivedOther>& m)
{
  derived().coeffs_nonconst() = m.coeffs();
  return derived();
}

template <typename _Derived>
template <typename _DerivedOther>
bool LieGroupBase<_Derived>::operator ==(
    const LieGroupBase<_DerivedOther>& m)
{
  return isApprox(m, Constants<Scalar>::eps);
}

template <typename _Derived>
template <typename _DerivedOther>
typename LieGroupBase<_Derived>::LieGroup
LieGroupBase<_Derived>::operator +(
    const TangentBase<_DerivedOther>& t) const
{
  return derived().rplus(t);
}

template <typename _Derived>
template <typename _DerivedOther>
_Derived&
LieGroupBase<_Derived>::operator +=(
    const TangentBase<_DerivedOther>& t)
{
  derived() = derived().rplus(t);
  return derived();
}

template <typename _Derived>
template <typename _DerivedOther>
typename LieGroupBase<_Derived>::Tangent
LieGroupBase<_Derived>::operator -(
    const LieGroupBase<_DerivedOther>& m) const
{
  return derived().rminus(m);
}

template <typename _Derived>
template <typename _DerivedOther>
typename LieGroupBase<_Derived>::LieGroup
LieGroupBase<_Derived>::operator *(
    const LieGroupBase<_DerivedOther>& m) const
{
  return derived().compose(m);
}

template <typename _Derived>
template <typename _DerivedOther>
_Derived&
LieGroupBase<_Derived>::operator *=(
    const LieGroupBase<_DerivedOther>& m)
{
  derived() = derived().compose(m);
  return derived();
}

/// Static helpers

template <typename _Derived>
typename LieGroupBase<_Derived>::LieGroup
LieGroupBase<_Derived>::Identity()
{
  const static LieGroup I(LieGroup().setIdentity());
  return I;
}

template <typename _Derived>
typename LieGroupBase<_Derived>::LieGroup
LieGroupBase<_Derived>::Random()
{
  return LieGroup().setRandom();
}

/// Utils

template <typename _Stream, typename _Derived>
_Stream& operator << (
    _Stream& s,
    const manif::LieGroupBase<_Derived>& m)
{
  s << m.coeffs().transpose();
  return s;
}

} /* namespace manif */

#endif /* _MANIF_MANIF_LIE_GROUP_BASE_H_ */
