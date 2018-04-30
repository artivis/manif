#ifndef _MANIF_MANIF_CERES_LOCAL_PARAMETRIZATION_H_
#define _MANIF_MANIF_CERES_LOCAL_PARAMETRIZATION_H_

/// @todo should I include them all ?
/// most likely
//#include "manif/impl/SO2_map.h"

#include <ceres/local_parameterization.h>

namespace manif
{

template <typename _Manifold>
class LocalParameterization
    : public ceres::LocalParameterization
{
  using Manifold = _Manifold;
  using Tangent  = typename _Manifold::Tangent;
  using Jacobian = typename _Manifold::Jacobian;

  /// @todo check diz J size :s
  using JacobianMap =
    Eigen::Map<Eigen::Matrix<
      double, Manifold::DoF, Manifold::DoF, Eigen::RowMajor>>;

public:

  LocalParameterization() : delta_zero_(Tangent::Zero()) {}
  virtual ~LocalParameterization() = default;

  /**
   * @brief Plus, rplus
   * @param state_raw
   * @param delta_raw
   * @param state_plus_delta_raw
   * @return true
   * @see SO2::rplus
   */
  virtual bool Plus(double const* state_raw,
                    double const* delta_raw,
                    double* state_plus_delta_raw) const override
  {
    const Eigen::Map<const Manifold> state(state_raw);
    const Eigen::Map<const Tangent>  delta(delta_raw);

    Eigen::Map<Manifold> state_plus_delta(state_plus_delta_raw);

    state_plus_delta = state + delta;

    return true;
  }

  /**
   * @brief ComputeJacobian
   * @param T_raw
   * @param jacobian_raw
   * @return
   * @see SO2::rplus
   */
  virtual bool ComputeJacobian(double const* state_raw,
                               double* rplus_jacobian_raw) const override
  {
    const Eigen::Map<const Manifold> state(state_raw);

    state.rplus(delta_zero_, tmp_out_, J_rplus_m, J_rplus_t);

    JacobianMap rplus_jacobian(rplus_jacobian_raw);
    rplus_jacobian = J_rplus_t;

    return true;
  }

  virtual int GlobalSize() const { return SO2d::RepSize; }
  virtual int LocalSize() const { return SO2d::DoF; }

protected:

  /*static*/ const Tangent delta_zero_;
  mutable Manifold tmp_out_;
  mutable Jacobian J_rplus_m, J_rplus_t;
};

//using LocalParameterizationSO2 = LocalParameterization<SO2d>;
//using LocalParameterizationSO3 = LocalParameterization<SO3d>;
//using LocalParameterizationSE2 = LocalParameterization<SE2d>;
//using LocalParameterizationSE3 = LocalParameterization<SE3d>;

} /* namespace manif */

#endif /* _MANIF_MANIF_CERES_LOCAL_PARAMETRIZATION_H_ */
