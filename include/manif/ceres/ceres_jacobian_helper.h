#ifndef _MANIF_MANIF_CERES_JACOBIAN_HELPER_H_
#define _MANIF_MANIF_CERES_JACOBIAN_HELPER_H_

#include "manif/SO2.h"
#include "manif/SE2.h"

namespace manif
{

template <typename _Manifold>
Eigen::Matrix<typename _Manifold::Scalar,
              _Manifold::RepSize,
              _Manifold::DoF>
computeLiftJacobianGlobal(const _Manifold& /*m*/)
{
  MANIF_NOT_IMPLEMENTED_YET;
  return Eigen::Matrix<typename _Manifold::Scalar,
                       _Manifold::RepSize,
                       _Manifold::DoF>();
}

template<>
Eigen::Matrix<SO2d::Scalar,
              SO2d::RepSize,
              SO2d::DoF>
computeLiftJacobianGlobal<Eigen::Map<const SO2d>>(
    const Eigen::Map<const SO2d>& m)
{
  using Jacobian_lift_coeffs = Eigen::Matrix<SO2d::Scalar,
                                             SO2d::RepSize,
                                             SO2d::DoF>;
  return (Jacobian_lift_coeffs()
           << -m.imag(),
               m.real() ).finished();
}

//template<>
//Eigen::Matrix<SO2d::Scalar,
//              SO2d::RepSize,
//              SO2d::DoF>
//computeLiftJacobianGlobal<Eigen::Map<const SE2d>>(
//    const Eigen::Map<const SE2d>& m)
//{
//  using Jacobian_lift_coeffs = Eigen::Matrix<SO2d::Scalar,
//                                             SO2d::RepSize,
//                                             SO2d::DoF>;
//  return (Jacobian_lift_coeffs()
//           <<  0,
//               0,
//              -m.imag(),
//               m.real() ).finished();
//}

} /* namespace manif */

#endif /* _MANIF_MANIF_CERES_JACOBIAN_HELPER_H_ */
