#ifndef _MANIF_MANIF_INTERPOLATION_H_
#define _MANIF_MANIF_INTERPOLATION_H_

#include "manif/impl/lie_group_base.h"

namespace manif {

/**
 * @brief Constexpr function to compute binomial coefficient.
 */
template <typename T>
constexpr T binomialCoeff(const T n, const T k) {
  return (n >= k) ?
    (k >= 0) ?
      (k*2 > n) ?
        binomialCoeff(n, n-k) :
        k ?
          binomialCoeff(n, k - 1) * (n - k + 1) / k :
          1
            // assert n ≥ k ≥ 0
      : (detail::raise<invalid_argument>("k >= 0 !"), T(0))
    : (detail::raise<invalid_argument>("n >= k !"), T(0));
}

/**
 * @brief Constexpr function to compute power.
 */
template <typename T>
constexpr T cpow(const T base, const int exp, T carry = 1) {
  return exp < 1 ? carry : cpow(base*base, exp/2, (exp % 2) ? carry*base : carry);
}

/**
 * @brief Constexpr function to compute the Bernstein polynomial
 */
template <typename T>
constexpr T polynomialBernstein(const T n, const T i, const T t) {
  return binomialCoeff(n, i) * cpow(T(1)-t, n-i) * cpow(t, i);
}

/**
 * @brief
 */
template <typename T>
T sphi(const T t, const std::size_t degree) {
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
template <typename _Derived>
static typename LieGroupBase<_Derived>::LieGroup
interpolateSlerp(
  const LieGroupBase<_Derived>& ma,
  const LieGroupBase<_Derived>& mb,
  const typename _Derived::Scalar t
  /*,
  typename LieGroupBase<_Derived>::OptJacobianRef J_mc_ma = {},
  typename LieGroupBase<_Derived>::OptJacobianRef J_mc_mb = {}
  */
)
{
  using Scalar = typename _Derived::Scalar;
  MANIF_CHECK(t >= Scalar(0) && t <= Scalar(1), "s must be be in [0, 1].");

  return ma + (t * (mb - ma));

  // using LieGroup = typename LieGroupBase<_Derived>::LieGroup;
//  using Jacobian = typename LieGroupBase<_Derived>::Jacobian;

  // LieGroup mc;

  // const auto _ = LieGroupBase<_Derived>::_;

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
  // {
  //   mc = ma.rplus( mb.rminus(ma) * t );
  // }

  // return mc;
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
interpolateCubic(
  const LieGroupBase<_Derived>& ma,
  const LieGroupBase<_Derived>& mb,
  const _Scalar t,
  const typename LieGroupBase<_Derived>::Tangent& ta =
    LieGroupBase<_Derived>::Tangent::Zero(),
  const typename LieGroupBase<_Derived>::Tangent& tb =
    LieGroupBase<_Derived>::Tangent::Zero()
  /*,
  typename LieGroupBase<_Derived>::OptJacobianRef J_mc_ma = {},
  typename LieGroupBase<_Derived>::OptJacobianRef J_mc_mb = {}
  */
) {
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
template <typename _Derived>
typename LieGroupBase<_Derived>::LieGroup
interpolateSmooth(
  const LieGroupBase<_Derived>& ma,
  const LieGroupBase<_Derived>& mb,
  const typename _Derived::Scalar t,
  const unsigned int m,
  const typename LieGroupBase<_Derived>::Tangent& ta =
    LieGroupBase<_Derived>::Tangent::Zero(),
  const typename LieGroupBase<_Derived>::Tangent& tb =
    LieGroupBase<_Derived>::Tangent::Zero()
  /*,
  typename LieGroupBase<_Derived>::OptJacobianRef J_mc_ma = {},
  typename LieGroupBase<_Derived>::OptJacobianRef J_mc_mb = {}
  */
) {
  using Scalar = typename LieGroupBase<_Derived>::Scalar;
//  using LieGroup = typename LieGroupBase<_Derived>::LieGroup;
//  using Jacobian = typename LieGroupBase<_Derived>::Jacobian;

  MANIF_CHECK(m >= Scalar(1), "m >= 1 !");
  MANIF_CHECK(t >= Scalar(0) && t <= Scalar(1), "t must be in [0, 1].");

  // with lplus

//  const auto r = mb.lplus(tb*(t-Scalar(1)));
//  const auto l = ma.lplus(ta*t);

  // with rplus

  const auto r = mb.rplus(tb * (t - Scalar(1)));
  const auto l = ma.rplus(ta * t);

  const auto B = r.lminus(l);

  return l.lplus(B * sphi(t, m));
}

enum class INTERP_METHOD
{
  SLERP,
  CUBIC,
  CNSMOOTH,
};

/**
 * @brief A helper function for interpolation.
 * @see interpolateSlerp.
 * @see interpolateCubic.
 * @see interpolateSmooth.
 */
template <typename _Derived, typename _Scalar>
typename LieGroupBase<_Derived>::LieGroup
interpolate(
  const LieGroupBase<_Derived>& ma,
  const LieGroupBase<_Derived>& mb,
  const _Scalar t,
  const INTERP_METHOD method = INTERP_METHOD::SLERP,
  const typename LieGroupBase<_Derived>::Tangent& ta =
    LieGroupBase<_Derived>::Tangent::Zero(),
  const typename LieGroupBase<_Derived>::Tangent& tb =
    LieGroupBase<_Derived>::Tangent::Zero()
  /*,
  typename LieGroupBase<_Derived>::OptJacobianRef J_mc_ma = {},
  typename LieGroupBase<_Derived>::OptJacobianRef J_mc_mb = {}
  */
) {
  switch (method) {
  case INTERP_METHOD::SLERP:
    return interpolateSlerp(ma, mb, t/*, J_mc_ma, J_mc_mb*/);
  case INTERP_METHOD::CUBIC:
    return interpolateCubic(ma, mb, t, ta, tb/*, J_mc_ma, J_mc_mb*/);
  case INTERP_METHOD::CNSMOOTH:
    return interpolateSmooth(ma, mb, t, 3, ta, tb/*, J_mc_ma, J_mc_mb*/);
  default:
    MANIF_THROW("Unknown interpolation method!");
    break;
  }

  return typename LieGroupBase<_Derived>::LieGroup();
}

} // namespace manif

#endif // _MANIF_MANIF_INTERPOLATION_H_
