#ifndef _MANIF_MANIF_UTILS_H_
#define _MANIF_MANIF_UTILS_H_

#include "manif/constants.h"

namespace manif
{

template <typename T>
T pi2pi(T angle)
{
  while (angle > T(M_PI))   angle -= T(2. * M_PI);
  while (angle <= T(-M_PI)) angle += T(2. * M_PI);

  return angle;
}

/**
 * @brief Conversion to radians
 * @param deg angle in degrees
 * @return angle in radians
 */
template<typename T>
constexpr T toRad(const T deg)
{
  return deg * Constants<T>::to_rad;
}

/**
 * @brief Conversion to degrees
 * @param rad angle in radians
 * @return angle in degrees
 */
template<typename T>
constexpr T toDeg(const T rad)
{
  return rad * Constants<T>::to_deg;
}

} /* namespace manif */

#endif /* _MANIF_MANIF_UTILS_H_ */
