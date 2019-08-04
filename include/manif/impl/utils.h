#ifndef _MANIF_MANIF_UTILS_H_
#define _MANIF_MANIF_UTILS_H_

#include "manif/constants.h"

namespace manif {

/**
 * @brief Wrap an angle in -pi,pi.
 * @param[in] angle The angle to be wrapped in radians
 * @return The wrapped angle.
 */
template <typename T>
T pi2pi(T angle)
{
  while (angle > T(MANIF_PI))   angle -= T(2. * MANIF_PI);
  while (angle <= T(-MANIF_PI)) angle += T(2. * MANIF_PI);

  return angle;
}

/**
 * @brief Conversion to radians
 * @param[in] deg angle in degrees
 * @return angle in radians
 */
template<typename T>
constexpr T toRad(const T deg)
{
  return deg * Constants<T>::to_rad;
}

/**
 * @brief Conversion to degrees
 * @param[in] rad angle in radians
 * @return angle in degrees
 */
template<typename T>
constexpr T toDeg(const T rad)
{
  return rad * Constants<T>::to_deg;
}

/**
 * @brief Degree 2 polynomial approximation of 1/sqrt(x) (reciprocal sqrt).
 * @param[in] x
 * @return ~1/sqrt(x)
 */
template <typename T>
constexpr T approxSqrtInv(const T x)
{
  return (T(15) / T(8)) - (T(5) / T(4)) * x + (T(3) / T(8)) * x * x;
}

} /* namespace manif */

#endif /* _MANIF_MANIF_UTILS_H_ */
