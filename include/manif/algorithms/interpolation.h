#ifndef _MANIF_MANIF_INTERPOLATION_H_
#define _MANIF_MANIF_INTERPOLATION_H_

#include "manif/impl/lie_group_base.h"

namespace manif {

/**
 * @brief Constexpr function to compute binomial coefficient.
 */
template <typename T>
constexpr T binomial_coefficient(const T n, const T k)
{
  return (n >= k) ? (k >= 0) ?
  (k*2 > n) ? binomial_coefficient(n, n-k) :
                     k ? binomial_coefficient(n, k - 1) * (n - k + 1) / k : 1
  // assert n ≥ k ≥ 0
  : (throw std::logic_error("k >= 0 !")) : (throw std::logic_error("n >= k !"));
}

/**
 * @brief Constexpr function to compute power.
 */
template <typename T>
constexpr T ipow(const T base, const int exp, T carry = 1) {
  return exp < 1 ? carry : ipow(base*base, exp/2, (exp % 2) ? carry*base : carry);
}

/**
 * @brief Constexpr function to compute the Bernstein polynomial
 */
template <typename T>
constexpr T polynomialBernstein(const T n, const T i, const T t)
{
  return binomial_coefficient(n, i) * ipow(T(1)-t, n-i) * ipow(t,i);
}

/**
 * @brief
 */
template <typename T>
T smoothing_phi(const T t, const std::size_t degree)
{
//  if (degree < 5)
//  {

    const T t2 = t*t;
    const T t3 = t2*t;
    const T t4 = t3*t;
    const T t5 = t4*t;
    const T t6 = t5*t;
    const T t7 = t6*t;
    const T t8 = t7*t;
    const T t9 = t8*t;

    return degree == 1 ? (T(3.)  *t2 - T(2.)  *t3)                                      :
           degree == 2 ? (T(10.) *t3 - T(15.) *t4 + T(6.)  *t5)                         :
           degree == 3 ? (T(35.) *t4 - T(84.) *t5 + T(70.) *t6 - T(20.) *t7)            :
           degree == 4 ? (T(126.)*t5 - T(420.)*t6 + T(540.)*t7 - T(315.)*t8 + T(70.)*t9):
           (throw std::logic_error("Not implemented yet !"));

//  }

//  T sum = 0;
//  T sum_gamma = 0;

//  for (std::size_t i=0; i<=degree; ++i)
//  {
//    const T am = (i % 2 == 0? T(1.) : T(-1.)) * binomial_coefficient(degree, i);

//    sum_gamma += (am / (degree + 1. + i));

//    sum += ((am / (degree + 1. + i)) * ipow(t, degree + 1. + i));
//  }

//  return (double(1) / sum_gamma) * sum;
}

/**
 * @brief Slerp interpolation.
 * @detail Interpolate a point mc between ma and mb at t in [0,1].
 * mc=ma if t=0
 * mc=mb if t=1
 * @param[in] ma Initial point.
 * @param[in] mb Final Point.
 * @param[in] t Time at which to interpolate in [0,1].
 * @param[in] -optional- J_mc_ma Jacobian of the interpolated point wrt ma.
 * @param[in] -optional- J_mc_mb Jacobian of the interpolated point wrt mb.
 */
template <typename _Derived, typename _Scalar>
static typename LieGroupBase<_Derived>::LieGroup
interpolate_slerp(const LieGroupBase<_Derived>& ma,
                  const LieGroupBase<_Derived>& mb,
                  const _Scalar t/*,
                  typename LieGroupBase<_Derived>::OptJacobianRef J_mc_ma =
                    LieGroupBase<_Derived>::_,
                  typename LieGroupBase<_Derived>::OptJacobianRef J_mc_mb =
                    LieGroupBase<_Derived>::_*/)
{
  MANIF_CHECK(t >= _Scalar(0) && t <= _Scalar(1),
              "s must be be in [0, 1].");

  using LieGroup = typename LieGroupBase<_Derived>::LieGroup;
//  using Jacobian = typename LieGroupBase<_Derived>::Jacobian;

  LieGroup mc;

  const auto _ = LieGroupBase<_Derived>::_;

  /// @todo optimize this
//  if (J_mc_ma && J_mc_mb)
//  {
//    Jacobian J_rmin_ma, J_rmin_mb;
//    Jacobian p1J_mc_ma;
//    Jacobian J_mc_rmin;

//    mc = ma.rplus( mb.rminus(ma, J_rmin_mb, J_rmin_ma) * t, p1J_mc_ma, J_mc_rmin);

//    (*J_mc_ma) = p1J_mc_ma + J_mc_rmin * (J_rmin_ma * t);
//    (*J_mc_mb) = J_mc_rmin * (J_rmin_mb * t);
//  }
//  else if (J_mc_ma)
//  {
//    Jacobian J_rmin_ma, p1J_mc_ma;
//    Jacobian J_mc_rmin;

//    mc = ma.rplus( mb.rminus(ma, _, J_rmin_ma) * t, p1J_mc_ma, J_mc_rmin);

//    (*J_mc_ma) = p1J_mc_ma + J_mc_rmin * (J_rmin_ma * t);
//  }
//  else if (J_mc_mb)
//  {
//    Jacobian J_rmin_mb, J_mc_rmin;

//    mc = ma.rplus( mb.rminus(ma, J_rmin_mb, _) * t, _, J_mc_rmin);

//    (*J_mc_mb) = J_mc_rmin * (J_rmin_mb * t);
//  }
//  else
  {
    mc = ma.rplus( mb.rminus(ma) * t );
  }

  return mc;
}

/**
 * @brief Cubic interpolation.
 * @detail Interpolate a point mc between ma and mb at t in [0,1].
 * mc=ma if t=0
 * mc=mb if t=1
 * @param[in] ma Initial point.
 * @param[in] mb Final Point.
 * @param[in] t Time at which to interpolate in [0,1].
 * @param[in] -optional- ta.
 * @param[in] -optional- tb.
 * @param[out] -optional- J_mc_ma Jacobian of the interpolated point wrt ma.
 * @param[out] -optional- J_mc_mb Jacobian of the interpolated point wrt mb.
 */
template <typename _Derived, typename _Scalar>
static typename LieGroupBase<_Derived>::LieGroup
interpolate_cubic(const LieGroupBase<_Derived>& ma,
                  const LieGroupBase<_Derived>& mb,
                  const _Scalar t,
                  const typename LieGroupBase<_Derived>::Tangent& ta =
                    LieGroupBase<_Derived>::Tangent::Zero(),
                  const typename LieGroupBase<_Derived>::Tangent& tb =
                    LieGroupBase<_Derived>::Tangent::Zero()/*,
                  typename LieGroupBase<_Derived>::OptJacobianRef J_mc_ma =
                    LieGroupBase<_Derived>::_,
                  typename LieGroupBase<_Derived>::OptJacobianRef J_mc_mb =
                    LieGroupBase<_Derived>::_*/)
{
  using Scalar   = typename LieGroupBase<_Derived>::Scalar;
  using LieGroup = typename LieGroupBase<_Derived>::LieGroup;
  //    using Jacobian = typename LieGroupBase<_Derived>::Jacobian;

  Scalar interp_factor(t);
  MANIF_CHECK(interp_factor >= Scalar(0) && interp_factor <= Scalar(1),
              "s must be be in [0, 1].");

  const Scalar t2 = t*t;
  const Scalar t3 = t2*t;

  LieGroup mc;

//  /// @todo optimize this
//  if (J_mc_ma && J_mc_mb)
//  {
//    /// @todo
//  }
//  else if (J_mc_ma)
//  {
//    /// @todo
//  }
//  else if (J_mc_mb)
//  {
//    /// @todo
//  }
//  else
  {
    const auto tab = mb.rminus(ma);
    //      const auto tba = ma.rminus(mb);

    const Scalar h00 =  Scalar(2)*t3 - Scalar(3)*t2 + Scalar(1);
    const Scalar h01 = -Scalar(2)*t3 + Scalar(3)*t2;
    const Scalar h10 =  t3 - Scalar(2)*t2 + t;
    const Scalar h11 =  t3 - t2;

    const auto l = ma.rplus(tab*h00).rplus(ta*h10);
    const auto r = mb.rplus(tab*(-h01)).rplus(tb*h11);
    const auto B = l.rminus(r);

    mc = r.rplus(B);
  }

  return mc;
}

/**
 * @brief Smooth interpolation.
 * @detail Interpolate a point mc between ma and mb at t in [0,1].
 * mc=ma if t=0
 * mc=mb if t=1
 * @param[in] ma Initial point.
 * @param[in] mb Final Point.
 * @param[in] t Time at which to interpolate in [0,1].
 * @param[in] -optional- ta.
 * @param[in] -optional- tb.
 * @param[out] -optional- J_mc_ma Jacobian of the interpolated point wrt ma.
 * @param[out] -optional- J_mc_mb Jacobian of the interpolated point wrt mb.
 *
 * @note "A two-step algorithm of smooth spline
 * generation on Riemannian manifolds",
 * Janusz Jakubiak and Fátima Silva Leite and Rui C. Rodrigues.
 */
template <typename _Derived, typename _Scalar>
static typename LieGroupBase<_Derived>::LieGroup
interpolate_smooth(const LieGroupBase<_Derived>& ma,
                   const LieGroupBase<_Derived>& mb,
                   const _Scalar t,
                   const unsigned int m,
                   const typename LieGroupBase<_Derived>::Tangent& ta =
                     LieGroupBase<_Derived>::Tangent::Zero(),
                   const typename LieGroupBase<_Derived>::Tangent& tb =
                     LieGroupBase<_Derived>::Tangent::Zero()/*,
                   typename LieGroupBase<_Derived>::OptJacobianRef J_mc_ma = LieGroupBase<_Derived>::_,
                   typename LieGroupBase<_Derived>::OptJacobianRef J_mc_mb = LieGroupBase<_Derived>::_*/)
{
  using Scalar   = typename LieGroupBase<_Derived>::Scalar;
//  using LieGroup = typename LieGroupBase<_Derived>::LieGroup;
//  using Jacobian = typename LieGroupBase<_Derived>::Jacobian;

  MANIF_CHECK(m >= Scalar(1), "m >= 1 !");
  Scalar interp_factor(t);
  MANIF_CHECK(interp_factor >= Scalar(0) && interp_factor <= Scalar(1),
              "s must be be in [0, 1].");

  const auto phi = smoothing_phi(t, m);

  // with lplus

//  const auto r = mb.lplus(tb*(t-Scalar(1)));
//  const auto l = ma.lplus(ta*t);

  // with rplus

  const auto r = mb.rplus(tb*(t-Scalar(1)));
  const auto l = ma.rplus(ta*t);

  const auto B = r.lminus(l);

  return l.lplus(B*phi);
}

enum class INTERP_METHOD
{
  SLERP,
  CUBIC,
  CNSMOOTH,
};

/**
 * @brief A helper function for interpolation.
 * @see interpolate_slerp.
 * @see interpolate_cubic.
 * @see interpolate_smooth.
 */
template <typename _Derived, typename _Scalar>
typename LieGroupBase<_Derived>::LieGroup
interpolate(const LieGroupBase<_Derived>& ma,
            const LieGroupBase<_Derived>& mb,
            const _Scalar t,
            const INTERP_METHOD method = INTERP_METHOD::SLERP,
            const typename LieGroupBase<_Derived>::Tangent& ta =
              LieGroupBase<_Derived>::Tangent::Zero(),
            const typename LieGroupBase<_Derived>::Tangent& tb =
              LieGroupBase<_Derived>::Tangent::Zero()/*,
            typename LieGroupBase<_Derived>::OptJacobianRef J_mc_ma =
              LieGroupBase<_Derived>::_,
            typename LieGroupBase<_Derived>::OptJacobianRef J_mc_mb =
              LieGroupBase<_Derived>::_*/)
{
  switch (method) {
  case INTERP_METHOD::SLERP:
    return interpolate_slerp(ma, mb, t/*, J_mc_ma, J_mc_mb*/);
  case INTERP_METHOD::CUBIC:
    return interpolate_cubic(ma, mb, t,
                             ta, tb/*,
                             J_mc_ma, J_mc_mb*/);
  case INTERP_METHOD::CNSMOOTH:
    return interpolate_smooth(ma, mb, t, 3,
                              ta, tb/*,
                              J_mc_ma, J_mc_mb*/);
  default:
    MANIF_THROW("Unknown interpolation method!");
    break;
  }

  return typename LieGroupBase<_Derived>::LieGroup();
}

} /* namespace manif */

#endif /* _MANIF_MANIF_INTERPOLATION_H_ */
