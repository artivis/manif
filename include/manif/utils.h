#ifndef _MANIF_MANIF_UTILS_H_
#define _MANIF_MANIF_UTILS_H_

#include <cmath>

namespace manif
{

template <typename T>
T pi2pi(T angle)
{
  while (angle > T(M_PI))   angle -= T(2. * M_PI);
  while (angle <= T(-M_PI)) angle += T(2. * M_PI);

  return angle;
}

} /* namespace manif */

#endif /* _MANIF_MANIF_UTILS_H_ */
