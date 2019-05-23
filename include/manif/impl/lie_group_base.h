#ifndef _MANIF_MANIF_LIE_GROUP_BASE_H_
#define _MANIF_MANIF_LIE_GROUP_BASE_H_

#include "manif/impl/macro.h"
#include "manif/impl/traits.h"
#include "manif/impl/eigen.h"
#include "manif/impl/tangent_base.h"

#include "manif/constants.h"

#include "lt/optional.hpp"

namespace manif {

/**
 * @brief Base class for Lie groups.
 * Defines the minimum common API.
 * @see TangentBase.
 */
template <class _Derived>
struct LieGroupBase
{
  static constexpr int Dim     = internal::traits<_Derived>::Dim;
  static constexpr int DoF     = internal::traits<_Derived>::DoF;
  static constexpr int RepSize = internal::traits<_Derived>::RepSize;

  using Scalar         = typename internal::traits<_Derived>::Scalar;
  using LieGroup       = typename internal::traits<_Derived>::LieGroup;
  using DataType       = typename internal::traits<_Derived>::DataType;
  using Tangent        = typename internal::traits<_Derived>::Tangent;
  using Jacobian       = typename internal::traits<_Derived>::Jacobian;
  using Vector         = typename internal::traits<_Derived>::Vector;

  using OptJacobianRef = tl::optional<Eigen::Ref<Jacobian>>;

  template <typename _Scalar>
  using LieGroupTemplate = typename internal::traitscast<LieGroup, _Scalar>::cast;

protected:

  //! @brief Access the underlying data by reference
  DataType& coeffs_nonconst();

public:

  //! @brief Helper for skipping an optional parameter.
  static const OptJacobianRef _;

  //! @brief Access the underlying data by const reference
  const DataType& coeffs() const;

  //! @brief Access the underlying data by pointer
  Scalar* data();
  //! @brief Access the underlying data by const pointer
  const Scalar* data() const;

  //! @brief Cast the LieGroup object to a copy
  //! of a different scalar type
  template <class _NewScalar>
  LieGroupTemplate<_NewScalar> cast() const;

  /// @todo 'cast' across groups
  /// SO3 so3 = so2.as<SO3>()
//  template <class _DerivedOther>
//  LieGroupTemplate<_DerivedOther> as() const;

  /**
   * @brief Set the LieGroup object this to Identity.
   * @return A reference to this.
   * @see Eq. (2).
   */
  _Derived& setIdentity();

  /**
   * @brief Set the LieGroup object this to a random value.
   * @return A reference to this.
   * @note Randomization happens in the tangent space so that
   * M = Log(tau.random)
   */
  _Derived& setRandom();

  // Minimum API
  // Those functions must be implemented in the Derived class !

  /**
   * @brief Get the inverse of the LieGroup object this.
   * @param[out] -optional-  J_m_t Jacobian of the inverse wrt this.
   * @return The Inverse of this.
   * @note See Eq. (3).
   * @see TangentBase.
   */
  LieGroup inverse(OptJacobianRef J_m_t = {}) const;

  /**
   * @brief Get the corresponding Lie algebra element in vector form.
   * @param[out] -optional- J_t_m Jacobian of the tangent wrt this.
   * @return The tangent element in vector form.
   * @note This is the log() map in vector form.
   * @see Eq. (24).
   */
  Tangent log(OptJacobianRef J_t_m = {}) const;

  /**
   * @brief This function is deprecated.
   * Please considere using
   * @ref log instead.
   */
  MANIF_DEPRECATED
  Tangent lift(OptJacobianRef J_t_m = {}) const;

  /**
   * @brief Composition of this and another element of the same Lie group.
   * @param[in]  m Another element of the same Lie group.
   * @param[out] -optional- J_mc_ma Jacobian of the composition wrt this.
   * @param[out] -optional- J_mc_mb Jacobian of the composition wrt m.
   * @return The composition of 'this . m'.
   * @note See Eqs. (1,2,3,4).
   */
  template <typename _DerivedOther>
  LieGroup compose(const LieGroupBase<_DerivedOther>& m,
                   OptJacobianRef J_mc_ma = {},
                   OptJacobianRef J_mc_mb = {}) const;

  /**
   * @brief TODO tofix
   * @param  v
   * @param[out] -optional- J_vout_m Jacobian of the new object wrt this.
   * @param[out] -optional- J_vout_v Jacobian of the new object wrt input object.
   * @return
   */
  Vector act(const Vector& v,
             OptJacobianRef J_vout_m = {},
             OptJacobianRef J_vout_v = {}) const;

  /**
   * @brief Get the Adjoint of the Lie group element this.
   * @note See Eq. (29).
   */
  Jacobian adj() const;

  // Deduced API

  /**
   * @brief Right oplus operation of the Lie group.
   * @param[in]  t An element of the tangent of the Lie group.
   * @param[out] -optional- J_mout_m Jacobian of the oplus operation wrt this.
   * @param[out] -optional- J_mout_t Jacobian of the oplus operation wrt the tangent element.
   * @return An element of the Lie group.
   * @note See Eq. (25).
   */
  template <typename _DerivedOther>
  LieGroup rplus(const TangentBase<_DerivedOther>& t,
                 OptJacobianRef J_mout_m = {},
                 OptJacobianRef J_mout_t = {}) const;

  /**
   * @brief Left oplus operation of the Lie group.
   * @param[in]  t An element of the tangent of the Lie group.
   * @param[out] -optional- J_mout_m Jacobian of the oplus operation wrt this.
   * @param[out] -optional- J_mout_t Jacobian of the oplus operation wrt the tangent element.
   * @return An element of the Lie group.
   * @note See Eq. (27).
   */
  template <typename _DerivedOther>
  LieGroup lplus(const TangentBase<_DerivedOther>& t,
                 OptJacobianRef J_mout_m = {},
                 OptJacobianRef J_mout_t = {}) const;

  /**
   * @brief An alias for the right oplus operation.
   * @see rplus
   */
  template <typename _DerivedOther>
  LieGroup plus(const TangentBase<_DerivedOther>& t,
                OptJacobianRef J_mout_m = {},
                OptJacobianRef J_mout_t = {}) const;

  /**
   * @brief Right ominus operation of the Lie group.
   * @param[in]  m Another element of the same Lie group.
   * @param[out] -optional- J_t_ma Jacobian of the ominus operation wrt this.
   * @param[out] -optional- J_t_mb Jacobian of the ominus operation wrt the other element.
   * @return An element of the tangent space of the Lie group.
   * @note See Eq. (26).
   */
  template <typename _DerivedOther>
  Tangent rminus(const LieGroupBase<_DerivedOther>& m,
                 OptJacobianRef J_t_ma = {},
                 OptJacobianRef J_t_mb = {}) const;

  /**
   * @brief Left ominus operation of the Lie group.
   * @param[in]  m Another element of the same Lie group.
   * @param[out] -optional- J_t_ma Jacobian of the ominus operation wrt this.
   * @param[out] -optional- J_t_mb Jacobian of the ominus operation wrt the other element.
   * @return An element of the tangent space of the Lie group.
   * @note See Eq. (28).
   */
  template <typename _DerivedOther>
  Tangent lminus(const LieGroupBase<_DerivedOther>& m,
                 OptJacobianRef J_t_ma = {},
                 OptJacobianRef J_t_mb = {}) const;

  /**
   * @brief An alias for the right ominus operation.
   * @see rminus
   */
  template <typename _DerivedOther>
  Tangent minus(const LieGroupBase<_DerivedOther>& m,
                OptJacobianRef J_t_ma = {},
                OptJacobianRef J_t_mb = {}) const;

  /**
   * @brief
   * @param[in]  m [description]
   * @param[out] -optional-  J_mc_ma [description]
   * @param[out] -optional-  J_mc_mb [description]
   * @return [description]
   */
  template <typename _DerivedOther>
  LieGroup between(const LieGroupBase<_DerivedOther>& m,
                   OptJacobianRef J_mc_ma = {},
                   OptJacobianRef J_mc_mb = {}) const;

  /**
   * @brief Evaluate whether this and m are 'close'.
   * @param[in] m An element of the same Lie Group.
   * @param[in] eps Threshold for equality copmarison.
   * @return true if the Lie group element m is 'close' to this,
   * false otherwise.
   * @see TangentBase::isApprox
   */
  template <typename _DerivedOther>
  bool isApprox(const LieGroupBase<_DerivedOther>& m,
                const Scalar eps = Constants<Scalar>::eps) const;

  // Some operators

  /**
   * @brief Assignment operator.
   * @param[in] An element of the Lie group.
   * @return A reference to this.
   */
  _Derived& operator =(const LieGroupBase<_Derived>& m);

  /**
   * @brief Assignment operator.
   * @param[in] An element of the Lie group.
   * @return A reference to this.
   */
  template <typename _DerivedOther>
  _Derived& operator =(const LieGroupBase<_DerivedOther>& m);

  /**
   * @brief Equality operator.
   * @param[in] An element of the same Lie group.
   * @return true if the Lie group element m is 'close' to this,
   * false otherwise.
   * @see isApprox.
   */
  template <typename _DerivedOther>
  bool operator ==(const LieGroupBase<_DerivedOther>& m);

  /**
   * @brief Right oplus operator.
   * @see rplus.
   */
  template <typename _DerivedOther>
  LieGroup operator +(const TangentBase<_DerivedOther>& t) const;

  /**
   * @brief Right in-place oplus operator.
   * @see rplus.
   */
  template <typename _DerivedOther>
  _Derived& operator +=(const TangentBase<_DerivedOther>& t);

  /**
   * @brief Right ominus operator.
   * @see rminus.
   */
  template <typename _DerivedOther>
  Tangent operator -(const LieGroupBase<_DerivedOther>& m) const;

  /**
   * @brief Lie group composition operator.
   * @see compose.
   */
  template <typename _DerivedOther>
  LieGroup operator *(const LieGroupBase<_DerivedOther>& m) const;

  /**
   * @brief Lie group in-place composition operator.
   * @see compose.
   */
  template <typename _DerivedOther>
  _Derived& operator *=(const LieGroupBase<_DerivedOther>& m);

  // Some static helpers

  //! Static helper to create a Lie group object set at Identity.
  static LieGroup Identity();
  //! Static helper to create a random object of the Lie group.
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
  derived() = zero.exp();
  return derived();
}

template <typename _Derived>
_Derived&
LieGroupBase<_Derived>::setRandom()
{
  internal::RandomEvaluator<
      typename internal::traits<_Derived>::Base>(
        derived()).run();

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
  if (J_mout_t)
  {
    (*J_mout_t) = t.rjac();
  }

  return compose(t.exp(), J_mout_m, _);
}

template <typename _Derived>
template <typename _DerivedOther>
typename LieGroupBase<_Derived>::LieGroup
LieGroupBase<_Derived>::lplus(
    const TangentBase<_DerivedOther>& t,
    OptJacobianRef J_mout_m,
    OptJacobianRef J_mout_t) const
{
  if (J_mout_t)
  {
    J_mout_t->noalias() = inverse().adj() * t.rjac();
  }

  if (J_mout_m)
  {
    J_mout_m->setIdentity();
  }

  return t.exp().compose(derived());
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
  const Tangent t = m.inverse().compose(derived()).log();

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
  const Tangent t = compose(m.inverse()).log();

  if (J_t_ma || J_t_mb)
  {
    const Jacobian J = t.rjacinv() * m.adj();

    if (J_t_ma)
    {
      (*J_t_ma) =  J;
    }
    if (J_t_mb)
    {
      (*J_t_mb) = -J;
    }
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
LieGroupBase<_Derived>::log(OptJacobianRef J_t_m) const
{
  return derived().log(J_t_m);
}

template <typename _Derived>
typename LieGroupBase<_Derived>::Tangent
LieGroupBase<_Derived>::lift(OptJacobianRef J_t_m) const
{
  return derived().log(J_t_m);
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
  const LieGroup mc = inverse().compose(m);

  if (J_mc_ma)
  {
    *J_mc_ma = -(mc.inverse().adj());
  }

  if (J_mc_mb)
  {
    J_mc_mb->setIdentity();
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

// Operators

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

// Static helpers

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

// Utils

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
