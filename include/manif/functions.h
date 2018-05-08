#ifndef _MANIF_MANIF_FUNCTIONS_H_
#define _MANIF_MANIF_FUNCTIONS_H_

#include "manif/impl/manifold_base.h"
//#include "manif/impl/utils.h"

namespace manif {

template <typename _Derived>
const typename _Derived::DataType&
coeffs(const ManifoldBase<_Derived>& manifold)
{
  return manifold.coeffs();
}

template <typename _Derived>
const typename _Derived::DataType&
coeffs(const TangentBase<_Derived>& tangent)
{
  return tangent.coeffs();
}

template <typename _Derived>
const typename _Derived::Scalar*
data(const ManifoldBase<_Derived>& manifold)
{
  return manifold.data();
}

template <typename _Derived>
typename _Derived::Scalar*
data(ManifoldBase<_Derived>& manifold)
{
  return manifold.data();
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
typename _Derived::Transformation
transform(const ManifoldBase<_Derived>& manifold)
{
  return manifold.transform();
}

template <typename _Derived>
typename _Derived::Rotation
rotation(const ManifoldBase<_Derived>& manifold)
{
  return manifold.rotation();
}

template <typename _Derived>
void
identity(ManifoldBase<_Derived>& manifold)
{
  manifold.identity();
}

template <typename _Manifold>
_Manifold Identity()
{
  return _Manifold::Identity();
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
random(ManifoldBase<_Derived>& manifold)
{
  manifold.random();
}

template <typename _Manifold>
_Manifold Random()
{
  return _Manifold::Random();
}

template <typename _Derived>
void
random(TangentBase<_Derived>& tangent)
{
  tangent.random();
}

template <typename Tangent>
Tangent Random()
{
  return Tangent::Random();
}

template <typename _Derived>
typename _Derived::Manifold
inverse(const ManifoldBase<_Derived>& manifold,
        typename _Derived::OpJacobianRef J_minv_m = {})
{
  return manifold.inverse(J_minv_m);
}

template <typename _DerivedMan, typename _DerivedTan>
typename _DerivedMan::Manifold
rplus(const ManifoldBase<_DerivedMan>& manifold,
      const TangentBase<_DerivedTan>& tangent,
      typename _DerivedMan::OpJacobianRef J_mout_m = {},
      typename _DerivedMan::OpJacobianRef J_mout_t = {})
{
  return manifold.rplus(tangent, J_mout_m, J_mout_t);
}

template <typename _DerivedMan, typename _DerivedTan>
typename _DerivedMan::Manifold
lplus(const ManifoldBase<_DerivedMan>& manifold,
      const TangentBase<_DerivedTan>& tangent,
      typename _DerivedMan::OpJacobianRef J_mout_m = {},
      typename _DerivedMan::OpJacobianRef J_mout_t = {})
{
  return manifold.lplus(tangent, J_mout_m, J_mout_t);
}

template <typename _DerivedMan, typename _DerivedTan>
typename _DerivedMan::Manifold
plus(const ManifoldBase<_DerivedMan>& manifold,
     const TangentBase<_DerivedTan>& tangent,
     typename _DerivedMan::OpJacobianRef J_mout_m = {},
     typename _DerivedMan::OpJacobianRef J_mout_t = {})
{
  return manifold.plus(tangent, J_mout_m, J_mout_t);
}

template <typename _Derived0, typename _Derived1>
typename _Derived0::Tangent
rminus(const ManifoldBase<_Derived0>& manifold_lhs,
       const ManifoldBase<_Derived1>& manifold_rhs,
       typename _Derived0::OptJacobianRef J_t_ma = {},
       typename _Derived0::OptJacobianRef J_t_mb = {})
{
  return manifold_lhs.rminus(manifold_rhs, J_t_ma, J_t_mb);
}

template <typename _Derived0, typename _Derived1>
typename _Derived0::Tangent
lminus(const ManifoldBase<_Derived0>& manifold_lhs,
       const ManifoldBase<_Derived1>& manifold_rhs,
       typename _Derived0::OptJacobianRef J_t_ma = {},
       typename _Derived0::OptJacobianRef J_t_mb = {})
{
  return manifold_lhs.lminus(manifold_rhs, J_t_ma, J_t_mb);
}

template <typename _Derived0, typename _Derived1>
typename _Derived0::Tangent
minus(const ManifoldBase<_Derived0>& manifold_lhs,
      const ManifoldBase<_Derived1>& manifold_rhs,
      typename _Derived0::OptJacobianRef J_t_ma = {},
      typename _Derived0::OptJacobianRef J_t_mb = {})
{
  return manifold_lhs.minus(manifold_rhs, J_t_ma, J_t_mb);
}

template <typename _Derived>
typename _Derived::Tangent
lift(const ManifoldBase<_Derived>& manifold,
     typename _Derived::OptJacobianRef J_l_m = {})
{
  return manifold.lift(J_l_m);
}

template <typename _Derived>
typename _Derived::Manifold
retract(const TangentBase<_Derived>& tangent,
        typename _Derived::OptJacobianRef J_r_t = {})
{
  return tangent.retract(J_r_t);
}

template <typename _Derived0, typename _Derived1>
typename _Derived0::Manifold
compose(const ManifoldBase<_Derived0>& manifold_lhs,
        const ManifoldBase<_Derived1>& manifold_rhs,
        typename _Derived0::OptJacobianRef J_mc_ma = {},
        typename _Derived0::OptJacobianRef J_mc_mb = {})
{
  return manifold_lhs.compose(manifold_rhs, J_mc_ma, J_mc_mb);
}

template <typename _Derived0, typename _Derived1>
typename _Derived0::Manifold
between(const ManifoldBase<_Derived0>& manifold_lhs,
        const ManifoldBase<_Derived1>& manifold_rhs,
        typename _Derived0::OptJacobianRef J_mc_ma = {},
        typename _Derived0::OptJacobianRef J_mc_mb = {})
{
  return manifold_lhs.between(manifold_rhs, J_mc_ma, J_mc_mb);
}

template <typename _Derived>
typename _Derived::Vector
act(const ManifoldBase<_Derived>& manifold,
    typename _Derived::Vector v,
    typename _Derived::OptJacobianRef J_vout_m = {},
    typename _Derived::OptJacobianRef J_vout_v = {})
{
  return manifold.act(v, J_vout_m, J_vout_v);
}

} /* namespace manif */

#endif /* _MANIF_MANIF_FUNCTIONS_H_ */
