#ifndef _MANIF_MANIF_FUNCTIONS_H_
#define _MANIF_MANIF_FUNCTIONS_H_

#include "manif/impl/lie_group_base.h"

namespace manif {

template <typename _Derived>
const typename _Derived::DataType&
coeffs(const LieGroupBase<_Derived>& lie_group)
{
  return lie_group.coeffs();
}

template <typename _Derived>
const typename _Derived::DataType&
coeffs(const TangentBase<_Derived>& tangent)
{
  return tangent.coeffs();
}

template <typename _Derived>
const typename _Derived::Scalar*
data(const LieGroupBase<_Derived>& lie_group)
{
  return lie_group.data();
}

template <typename _Derived>
typename _Derived::Scalar*
data(LieGroupBase<_Derived>& lie_group)
{
  return lie_group.data();
}

template <typename _Derived>
const typename _Derived::Scalar*
data(const TangentBase<_Derived>& tangent)
{
  return tangent.data();
}

template <typename _Derived>
typename _Derived::Scalar*
data(TangentBase<_Derived>& tangent)
{
  return tangent.data();
}

template <typename _Derived>
void
identity(LieGroupBase<_Derived>& lie_group)
{
  lie_group.identity();
}

template <typename _LieGroup>
_LieGroup Identity()
{
  return _LieGroup::Identity();
}

template <typename _Derived>
void
zero(TangentBase<_Derived>& tangent)
{
  tangent.zero();
}

template <typename _Tangent>
_Tangent Zero()
{
  return _Tangent::Zero();
}

template <typename _Derived>
void
random(LieGroupBase<_Derived>& lie_group)
{
  lie_group.random();
}

template <typename _Type>
_Type Random()
{
  return _Type::Random();
}

template <typename _Derived>
void
random(TangentBase<_Derived>& tangent)
{
  tangent.random();
}

template <typename _Derived>
typename _Derived::LieGroup
inverse(const LieGroupBase<_Derived>& lie_group,
        typename _Derived::OpJacobianRef J_minv_m = {})
{
  return lie_group.inverse(J_minv_m);
}

template <typename _DerivedMan, typename _DerivedTan>
typename _DerivedMan::LieGroup
rplus(const LieGroupBase<_DerivedMan>& lie_group,
      const TangentBase<_DerivedTan>& tangent,
      typename _DerivedMan::OpJacobianRef J_mout_m = {},
      typename _DerivedMan::OpJacobianRef J_mout_t = {})
{
  return lie_group.rplus(tangent, J_mout_m, J_mout_t);
}

template <typename _DerivedMan, typename _DerivedTan>
typename _DerivedMan::LieGroup
lplus(const LieGroupBase<_DerivedMan>& lie_group,
      const TangentBase<_DerivedTan>& tangent,
      typename _DerivedMan::OpJacobianRef J_mout_m = {},
      typename _DerivedMan::OpJacobianRef J_mout_t = {})
{
  return lie_group.lplus(tangent, J_mout_m, J_mout_t);
}

template <typename _DerivedMan, typename _DerivedTan>
typename _DerivedMan::LieGroup
plus(const LieGroupBase<_DerivedMan>& lie_group,
     const TangentBase<_DerivedTan>& tangent,
     typename _DerivedMan::OpJacobianRef J_mout_m = {},
     typename _DerivedMan::OpJacobianRef J_mout_t = {})
{
  return lie_group.plus(tangent, J_mout_m, J_mout_t);
}

template <typename _Derived0, typename _Derived1>
typename _Derived0::Tangent
rminus(const LieGroupBase<_Derived0>& lie_group_lhs,
       const LieGroupBase<_Derived1>& lie_group_rhs,
       typename _Derived0::OptJacobianRef J_t_ma = {},
       typename _Derived0::OptJacobianRef J_t_mb = {})
{
  return lie_group_lhs.rminus(lie_group_rhs, J_t_ma, J_t_mb);
}

template <typename _Derived0, typename _Derived1>
typename _Derived0::Tangent
lminus(const LieGroupBase<_Derived0>& lie_group_lhs,
       const LieGroupBase<_Derived1>& lie_group_rhs,
       typename _Derived0::OptJacobianRef J_t_ma = {},
       typename _Derived0::OptJacobianRef J_t_mb = {})
{
  return lie_group_lhs.lminus(lie_group_rhs, J_t_ma, J_t_mb);
}

template <typename _Derived0, typename _Derived1>
typename _Derived0::Tangent
minus(const LieGroupBase<_Derived0>& lie_group_lhs,
      const LieGroupBase<_Derived1>& lie_group_rhs,
      typename _Derived0::OptJacobianRef J_t_ma = {},
      typename _Derived0::OptJacobianRef J_t_mb = {})
{
  return lie_group_lhs.minus(lie_group_rhs, J_t_ma, J_t_mb);
}

template <typename _Derived>
MANIF_DEPRECATED
typename _Derived::Tangent
lift(const LieGroupBase<_Derived>& lie_group,
     typename _Derived::OptJacobianRef J_l_m = {})
{
  return lie_group.log(J_l_m);
}

template <typename _Derived>
typename _Derived::Tangent
log(const LieGroupBase<_Derived>& lie_group,
     typename _Derived::OptJacobianRef J_l_m = {})
{
  return lie_group.log(J_l_m);
}

template <typename _Derived>
MANIF_DEPRECATED
typename _Derived::LieGroup
retract(const TangentBase<_Derived>& tangent,
        typename _Derived::OptJacobianRef J_r_t = {})
{
  return tangent.exp(J_r_t);
}

template <typename _Derived>
typename _Derived::LieGroup
exp(const TangentBase<_Derived>& tangent,
    typename _Derived::OptJacobianRef J_e_t = {})
{
  return tangent.exp(J_e_t);
}

template <typename _Derived0, typename _Derived1>
typename _Derived0::LieGroup
compose(const LieGroupBase<_Derived0>& lie_group_lhs,
        const LieGroupBase<_Derived1>& lie_group_rhs,
        typename _Derived0::OptJacobianRef J_mc_ma = {},
        typename _Derived0::OptJacobianRef J_mc_mb = {})
{
  return lie_group_lhs.compose(lie_group_rhs, J_mc_ma, J_mc_mb);
}

template <typename _Derived0, typename _Derived1>
typename _Derived0::LieGroup
between(const LieGroupBase<_Derived0>& lie_group_lhs,
        const LieGroupBase<_Derived1>& lie_group_rhs,
        typename _Derived0::OptJacobianRef J_mc_ma = {},
        typename _Derived0::OptJacobianRef J_mc_mb = {})
{
  return lie_group_lhs.between(lie_group_rhs, J_mc_ma, J_mc_mb);
}

template <typename _Derived>
typename _Derived::Vector
act(const LieGroupBase<_Derived>& lie_group,
    typename _Derived::Vector v,
    typename _Derived::OptJacobianRef J_vout_m = {},
    typename _Derived::OptJacobianRef J_vout_v = {})
{
  return lie_group.act(v, J_vout_m, J_vout_v);
}

} /* namespace manif */

#endif /* _MANIF_MANIF_FUNCTIONS_H_ */
