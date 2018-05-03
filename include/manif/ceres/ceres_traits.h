#ifndef _MANIF_MANIF_CERES_TRAITS_H_
#define _MANIF_MANIF_CERES_TRAITS_H_

#include <Eigen/Core>

namespace manif
{

namespace internal
{

template <typename _Jacobian>
struct traits_ceres;

template <typename _Scalar, int _Rows, int _Cols>
struct traits_ceres<Eigen::Matrix<_Scalar, _Rows, _Cols>>
{
  using JacobianMap =
    Eigen::Map<
      Eigen::Matrix<_Scalar, _Rows, _Cols, Eigen::RowMajor>>;
};

} /* namespace internal */
} /* namespace manif */

#endif /* _MANIF_MANIF_CERES_TRAITS_H_ */
