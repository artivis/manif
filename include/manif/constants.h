#ifndef _MANIF_MANIF_CONSTANTS_H_
#define _MANIF_MANIF_CONSTANTS_H_

#include <cmath>
#include <limits>

namespace manif {
namespace internal {

double constexpr sqrtNewtonRaphson(double x, double curr, double prev)
{
  return curr == prev
          ? curr
          : sqrtNewtonRaphson(x, 0.5 * (curr + x / curr), curr);
}

/*
* Constexpr version of the square root
* Return value:
*   - For a finite and non-negative value of "x", returns an approximation for the square root of "x"
*   - Otherwise, returns NaN
*
* credits : https://stackoverflow.com/a/34134071/9709397
*/
double constexpr csqrt(double x)
{
  return x >= 0 && x < std::numeric_limits<double>::infinity()
        ? sqrtNewtonRaphson(x, x, 0)
        : std::numeric_limits<double>::quiet_NaN();
}

} /* namespace internal */

template <typename _Scalar>
struct Constants
{
  static constexpr _Scalar eps      = _Scalar(1e-10);
  static constexpr _Scalar eps_s    = _Scalar(1e-15); // ~
  static constexpr _Scalar eps_sqrt = internal::csqrt(eps);

  static constexpr _Scalar to_rad = _Scalar(M_PI / 180);
  static constexpr _Scalar to_deg = _Scalar(180.0 / M_PI);
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

} /* namespace manif  */

#endif /* _MANIF_MANIF_CONSTANTS_H_ */
