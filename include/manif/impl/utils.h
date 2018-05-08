#ifndef _MANIF_MANIF_UTILS_H_
#define _MANIF_MANIF_UTILS_H_

#include "manif/impl/eigen_checks.h"
#include <Eigen/Core>

namespace manif
{

template <typename T>
T pi2pi(T angle)
{
  while (angle > T(M_PI))   angle -= T(2. * M_PI);
  while (angle <= T(-M_PI)) angle += T(2. * M_PI);

  return angle;
}

template <typename _Scalar>
Eigen::Matrix<_Scalar, 2, 2>
skew2(const _Scalar v)
{
  return (Eigen::Matrix<_Scalar, 2, 2>() <<
             _Scalar(0.), -v,
             v, _Scalar(0.) ).finished();
}

template <typename _Derived>
Eigen::Matrix<typename _Derived::Scalar, 3, 3>
skew3(const Eigen::MatrixBase<_Derived>& v)
{
  assert_vector_dim(v, 3);

  using T = typename _Derived::Scalar;

  return (Eigen::Matrix<T, 3, 3>() <<
             T(0.),  -v(2),   +v(1),
            +v(2),    T(0.),  -v(0),
            -v(1),   +v(0),    T(0.) ).finished();
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
