#ifndef _MANIF_CONSTANTS_H_
#define _MANIF_CONSTANTS_H_

#include <cmath>

namespace manif
{

//template <typename _Scalar>
//struct Constants
//{
//  static constexpr _Scalar eps      = _Scalar(1e-10);
//  static constexpr _Scalar eps_sq   = eps*eps;
//  static constexpr _Scalar eps_sqrt = sqrt(eps);

//  static constexpr _Scalar to_rad =
//      _Scalar(0.017453292519943295769236907684886127134); // pi / 180
//  static constexpr _Scalar to_deg =
//      _Scalar(57.295779513082320876798154814105170332); // 180 / pi
//};

namespace internal
{
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
  static constexpr _Scalar eps_sq   = eps*eps ;
  static constexpr _Scalar eps_sqrt = internal::csqrt(eps);

  static constexpr _Scalar to_rad = _Scalar(M_PI / 180);
  static constexpr _Scalar to_deg = _Scalar(180.0 / M_PI);
};

} /* namespace manif  */

#endif /* _MANIF_CONSTANTS_H_ */
