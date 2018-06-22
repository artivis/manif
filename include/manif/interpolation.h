#ifndef _MANIF_MANIF_INTERPOLATION_H_
#define _MANIF_MANIF_INTERPOLATION_H_

#include "manif/impl/manifold_base.h"

namespace manif
{

enum class INTERP_METHOD
{
  SLERP,
  CUBIC,
  TWOSTEPS,
};

template <INTERP_METHOD Method>
struct Interpolater;

template <>
struct Interpolater<INTERP_METHOD::SLERP>
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
struct Interpolater<INTERP_METHOD::CUBIC>
{
  template <typename _Derived, typename _Scalar>
  static typename ManifoldBase<_Derived>::Manifold
  interp(const ManifoldBase<_Derived>& ma,
         const ManifoldBase<_Derived>& mb,
         const _Scalar t,
         typename ManifoldBase<_Derived>::OptJacobianRef J_mc_ma,
         typename ManifoldBase<_Derived>::OptJacobianRef J_mc_mb)
  {
    using Scalar   = typename ManifoldBase<_Derived>::Scalar;
    using Manifold = typename ManifoldBase<_Derived>::Manifold;
//    using Jacobian = typename ManifoldBase<_Derived>::Jacobian;

    Scalar interp_factor(t);
    MANIF_CHECK(interp_factor >= Scalar(0) && interp_factor <= Scalar(1),
                "s must be be in [0, 1].");

    const Scalar t2 = t*t;
    const Scalar t3 = t2*t;

    Manifold mc;

    /// @todo optimize this
    if (J_mc_ma && J_mc_mb)
    {
      /// @todo
    }
    else if (J_mc_ma)
    {
      /// @todo
    }
    else if (J_mc_mb)
    {
      /// @todo
    }
    else
    {
      const auto ta  = ma.lift();
      const auto tb  = mb.lift();
      const auto tab = mb.rminus(ma);
//      const auto tba = ma.rminus(mb);

      const Scalar h00 =  Scalar(2)*t3 - Scalar(3)*t2 + Scalar(1);
      const Scalar h01 = -Scalar(2)*t3 + Scalar(3)*t2;
      const Scalar h10 =  t3 - Scalar(2)*t2 + t;
      const Scalar h11 =  t3 - t2;

      mc = (ta*h00 + tb*h01 + tab*h10 + tab*h11).retract();
    }

    return mc;
  }
};

/**
 * @note "A two-step algorithm of smooth spline
 * generation on Riemannian manifolds",
 * Janusz Jakubiak and Fátima Silva Leite and Rui C. Rodrigues.
 */
template <>
struct Interpolater<INTERP_METHOD::TWOSTEPS>
{
  template <typename _Derived, typename _Scalar>
  static typename ManifoldBase<_Derived>::Manifold
  interp(const ManifoldBase<_Derived>& ma,
         const ManifoldBase<_Derived>& mb,
         const _Scalar t,
         const int m = 2,
         typename ManifoldBase<_Derived>::OptJacobianRef J_mc_ma = ManifoldBase<_Derived>::_,
         typename ManifoldBase<_Derived>::OptJacobianRef J_mc_mb = ManifoldBase<_Derived>::_)
  {
    using Scalar   = typename ManifoldBase<_Derived>::Scalar;
    using Manifold = typename ManifoldBase<_Derived>::Manifold;
//    using Jacobian = typename ManifoldBase<_Derived>::Jacobian;

    Scalar interp_factor(t);
    MANIF_CHECK(interp_factor >= Scalar(0) && interp_factor <= Scalar(1),
                "s must be be in [0, 1].");

    const Scalar t2 = t*t;
    const Scalar t3 = t2*t;
    const Scalar t4 = t3*t;
    const Scalar t5 = t4*t;
    const Scalar t6 = t5*t;
    const Scalar t7 = t6*t;
    const Scalar t8 = t7*t;
    const Scalar t9 = t8*t;

    Scalar psi;

    switch (m) {
    case 1:
      psi = Scalar(3)*t2 - Scalar(2)*t3;
      break;
    case 2:
      psi = Scalar(10)*t3 - Scalar(15)*t4 + Scalar(6)*t5;
      break;
    case 3:
      psi = Scalar(35)*t4 - Scalar(84)*t5 + Scalar(70)*t6 - Scalar(20)*t7;
      break;
    case 4:
      psi = Scalar(126)*t5 - Scalar(420)*t6 + Scalar(540)*t7 - Scalar(315)*t8 + Scalar(70)*t9;
      break;
    default:
      // m = 2
      psi = Scalar(10)*t3 - Scalar(15)*t4 + Scalar(6)*t5;
      break;
    }

    Manifold mc;

    const auto ta = ma.lift();
    const auto tb = mb.lift();

    const auto l = (ta*t).retract().compose(ma);
    const auto r = (tb*(t-Scalar(1))).retract().compose(mb);
    const auto B = r.lminus(l);

    mc = (B*psi).retract().compose(l);

//    const auto ta = ma.lift();
//    const auto tb = mb.lift();

//    const auto l = ma.lplus(ta*t);
//    const auto r = mb.lplus(tb*(t-Scalar(1)));
//    const auto B = r.lminus(l);

//    mc = l.lplus(B*psi);

    return mc;
  }
};

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
    return Interpolater<INTERP_METHOD::SLERP>::interp(ma, mb, t, J_mc_ma, J_mc_mb);
  case INTERP_METHOD::CUBIC:
    return Interpolater<INTERP_METHOD::CUBIC>::interp(ma, mb, t, J_mc_ma, J_mc_mb);
  case INTERP_METHOD::TWOSTEPS:
    return Interpolater<INTERP_METHOD::TWOSTEPS>::interp(ma, mb, t, 2, J_mc_ma, J_mc_mb);
  default:
    break;
  }

  return typename ManifoldBase<_Derived>::Manifold();
}

} /* namespace manif */

#endif /* _MANIF_MANIF_INTERPOLATION_H_ */
