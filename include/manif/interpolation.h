#ifndef _MANIF_MANIF_INTERPOLATION_H_
#define _MANIF_MANIF_INTERPOLATION_H_

#include "manif/impl/manifold_base.h"

namespace manif
{

enum class INTERP_METHOD
{
  SLERP,
  CUBIC
};

namespace detail {

template <INTERP_METHOD Method>
struct InterpolationHelper;

template <>
struct InterpolationHelper<INTERP_METHOD::SLERP>
{
  template <typename _Derived, typename _Scalar>
  static typename ManifoldBase<_Derived>::Manifold
  interp(const ManifoldBase<_Derived>& ma,
         const ManifoldBase<_Derived>& mb,
         const _Scalar t,
         typename ManifoldBase<_Derived>::OptJacobianRef J_mc_ma = ManifoldBase<_Derived>::_,
         typename ManifoldBase<_Derived>::OptJacobianRef J_mc_mb = ManifoldBase<_Derived>::_)
  {
    MANIF_CHECK(t >= _Scalar(0) && t <= _Scalar(1),
                "s must be be in [0, 1].");

    using Manifold = typename ManifoldBase<_Derived>::Manifold;
    using Jacobian = typename ManifoldBase<_Derived>::Jacobian;

    Manifold mc;

    const auto _ = ManifoldBase<_Derived>::_;

    /// @todo optimize this
    if (J_mc_ma && J_mc_mb)
    {
      Jacobian J_rmin_ma, J_rmin_mb;
      Jacobian p1J_mc_ma;
      Jacobian J_mc_rmin;

      mc = ma.rplus( mb.rminus(ma, J_rmin_mb, J_rmin_ma) * t, p1J_mc_ma, J_mc_rmin);

      (*J_mc_ma) = p1J_mc_ma + J_mc_rmin * J_rmin_ma;
      (*J_mc_mb) = J_mc_rmin * J_rmin_mb;
    }
    else if (J_mc_ma)
    {
      Jacobian J_rmin_ma, p1J_mc_ma;
      Jacobian J_mc_rmin;

      mc = ma.rplus( mb.rminus(ma, _, J_rmin_ma) * t, p1J_mc_ma, J_mc_rmin);

      (*J_mc_ma) = p1J_mc_ma + J_mc_rmin * J_rmin_ma;
    }
    else if (J_mc_mb)
    {
      Jacobian J_rmin_mb, J_mc_rmin;

      mc = ma.rplus( mb.rminus(ma, J_rmin_mb, _) * t, _, J_mc_rmin);

      (*J_mc_mb) = J_mc_rmin * J_rmin_mb;
    }
    else
    {
      mc = ma.rplus( mb.rminus(ma) * t );
    }

    return mc;
  }
};

template <>
struct InterpolationHelper<INTERP_METHOD::CUBIC>
{
  template <typename _Derived, typename _Scalar>
  static typename ManifoldBase<_Derived>::Manifold
  interp(const ManifoldBase<_Derived>& ma,
         const ManifoldBase<_Derived>& mb,
         const _Scalar t,
         typename ManifoldBase<_Derived>::OptJacobianRef J_mc_ma,
         typename ManifoldBase<_Derived>::OptJacobianRef J_mc_mb)
  {
    _Scalar interp_factor(t);
    MANIF_CHECK(interp_factor >= _Scalar(0) && interp_factor <= _Scalar(1),
                "s must be be in [0, 1].");

//    const _Scalar t2 = t*t;
//    const _Scalar t3 = t2*t;

    using Manifold = typename ManifoldBase<_Derived>::Manifold;
//    using Jacobian = typename ManifoldBase<_Derived>::Jacobian;

    Manifold mc;

//    /// @todo optimize this
//    if (J_mc_ma && J_mc_mb)
//    {
//      Jacobian J_rmin_ma, J_rmin_mb;
//      Jacobian J_ret_rmin;
//      Jacobian J_mc_ret, p1J_mc_ma;

//      mc = ma.compose(
//        (mb.rminus(ma, J_rmin_mb, J_rmin_ma) * interp_factor).retract(J_ret_rmin),
//          p1J_mc_ma, J_mc_ret );

//      (*J_mc_ma) = p1J_mc_ma + J_mc_ret * J_ret_rmin * J_rmin_ma;
//      (*J_mc_mb) = J_mc_ret * J_ret_rmin * J_rmin_mb;
//    }
//    else if (J_mc_ma)
//    {
//      Jacobian J_rmin_ma;
//      Jacobian J_ret_rmin;
//      Jacobian J_mc_ret, p1J_mc_ma;

//      mc = ma.compose(
//        (mb.rminus(ma, ManifoldBase<_Derived>::_, J_rmin_ma) * interp_factor).retract(J_ret_rmin),
//          p1J_mc_ma, J_mc_ret );

//      (*J_mc_ma) = p1J_mc_ma + J_mc_ret * J_ret_rmin * J_rmin_ma;
//    }
//    else if (J_mc_mb)
//    {
//      Jacobian J_rmin_mb;
//      Jacobian J_ret_rmin;
//      Jacobian J_mc_ret;

//      mc = ma.compose(
//        (mb.rminus(ma, J_rmin_mb, ManifoldBase<_Derived>::_) * interp_factor).retract(J_ret_rmin),
//          ManifoldBase<_Derived>::_, J_mc_ret );

//      (*J_mc_mb) = J_mc_ret * J_ret_rmin * J_rmin_mb;
//    }
//    else
//    {
//      const auto ta = ma.lift();
//      const auto tb = mb.lift();

//      mc = ((mb.rminus(ma) * (_Scalar(3)*t2 - _Scalar(2)*t3)) +
//            ta * (t3 - _Scalar(2)*t2 + t) +
//            tb * (t3 - t2)).lift();
//    }

    return mc;
  }
};

} /* namespace detail */

template <typename _Derived, typename _Scalar>
typename ManifoldBase<_Derived>::Manifold
interpolate(const ManifoldBase<_Derived>& ma,
            const ManifoldBase<_Derived>& mb,
            const _Scalar t,
            const INTERP_METHOD method = INTERP_METHOD::SLERP,
            typename ManifoldBase<_Derived>::OptJacobianRef J_mc_ma = ManifoldBase<_Derived>::_,
            typename ManifoldBase<_Derived>::OptJacobianRef J_mc_mb = ManifoldBase<_Derived>::_)
{
  switch (method) {
  case INTERP_METHOD::SLERP:
    return detail::InterpolationHelper<INTERP_METHOD::SLERP>::interp(ma, mb, t, J_mc_ma, J_mc_mb);
  case INTERP_METHOD::CUBIC:
    return detail::InterpolationHelper<INTERP_METHOD::CUBIC>::interp(ma, mb, t, J_mc_ma, J_mc_mb);
  default:
    break;
  }

  return typename ManifoldBase<_Derived>::Manifold();
}

} /* namespace manif */

#endif /* _MANIF_MANIF_INTERPOLATION_H_ */
