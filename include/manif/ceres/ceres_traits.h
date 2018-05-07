#ifndef _MANIF_MANIF_CERES_TRAITS_H_
#define _MANIF_MANIF_CERES_TRAITS_H_

#include "manif/impl/manifold_base.h"
#include <Eigen/Core>

namespace manif
{
namespace internal
{

template <typename _Manifold>
struct traits_ceres
{
  using Manifold = _Manifold;

  using JacobianMap =
    Eigen::Map<
      Eigen::Matrix<typename Manifold::Scalar,
                    Manifold::RepSize,
                    Manifold::DoF/*,
                    Eigen::RowMajor*/>>;
};

} /* namespace internal */
} /* namespace manif */

#endif /* _MANIF_MANIF_CERES_TRAITS_H_ */
