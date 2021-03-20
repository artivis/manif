#ifndef _MANIF_MANIF_CONSTANTS_H_
#define _MANIF_MANIF_CONSTANTS_H_

#include <cmath>
#include <limits>

#define MANIF_PI   3.141592653589793238462643383279502884
#define MANIF_PI_2 1.570796326794896619231321691639751442
#define MANIF_PI_4 0.785398163397448309615660845819875721

namespace manif {
namespace internal {

/**
 * Constexpr Newton-Raphson iterative algorithm for the sqrt aprrox.
 */
template <typename T>
T constexpr sqrtNewtonRaphson(T x, T curr, T prev)
{
  return curr == prev
          ? curr
          : sqrtNewtonRaphson(x, T(0.5) * (curr + x / curr), curr);
}

/**
 * Constexpr version of the square root
 * Return value:
 *   - For a finite and non-negative value of "x",
 *     returns an approximation for the square root of "x"
 *   - Otherwise, returns NaN
 *
 * credits : https://stackoverflow.com/a/34134071/9709397
 */
template <typename T>
T constexpr csqrt(T x)
{
  return x >= T(0) && x < std::numeric_limits<T>::infinity()
        ? sqrtNewtonRaphson(x, x, T(0))
        : std::numeric_limits<T>::quiet_NaN();
}

} /* namespace internal */

/**
 * @brief Traits to define some constant scalar.
 */
template <typename _Scalar>
struct Constants
{
  static constexpr _Scalar eps      = _Scalar(1e-10);
  static constexpr _Scalar eps_s    = _Scalar(1e-15); // ~
  static constexpr _Scalar eps_sqrt = internal::csqrt(eps);

  static constexpr _Scalar to_rad = _Scalar(MANIF_PI / 180.0);
  static constexpr _Scalar to_deg = _Scalar(180.0 / MANIF_PI);
};

template <typename _Scalar>
constexpr _Scalar Constants<_Scalar>::eps;
template <typename _Scalar>
constexpr _Scalar Constants<_Scalar>::eps_s;
template <typename _Scalar>
constexpr _Scalar Constants<_Scalar>::eps_sqrt;
template <typename _Scalar>
constexpr _Scalar Constants<_Scalar>::to_rad;
template <typename _Scalar>
constexpr _Scalar Constants<_Scalar>::to_deg;

template <>
struct Constants<float>
{
  static constexpr float eps      = float(1e-5);
  static constexpr float eps_s    = float(1e-6); // ~
  static constexpr float eps_sqrt = internal::csqrt(eps);

  static constexpr float to_rad = float(MANIF_PI / 180.0);
  static constexpr float to_deg = float(180.0 / MANIF_PI);
};

} /* namespace manif  */

#endif /* _MANIF_MANIF_CONSTANTS_H_ */
