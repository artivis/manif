#ifndef _MANIF_MANIF_CERES_LOCAL_PARAMETRIZATION_H_
#define _MANIF_MANIF_CERES_LOCAL_PARAMETRIZATION_H_

/// @todo should I include them all ?
/// most likely
//#include "manif/impl/SO2_map.h"

#include <ceres/local_parameterization.h>

namespace manif
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

template <typename _Manifold>
class LocalParameterization
    : public ceres::LocalParameterization
{
  using Manifold = _Manifold;
  using Tangent  = typename _Manifold::Tangent;
  using Jacobian = typename _Manifold::Jacobian;

  using JacobianMap = typename traits_ceres<Jacobian>::JacobianMap;

  template <typename _Scalar>
  using ManifoldTemplate =
  typename _Manifold::template ManifoldTemplate<_Scalar>;

  template <typename _Scalar>
  using TangentTemplate =
  typename Tangent::template TangentTemplate<_Scalar>;

public:

  LocalParameterization() : tangent_zero_(Tangent::Zero()) {}
  virtual ~LocalParameterization() = default;

  template<typename T>
  bool operator()(const T* state_raw,
                  const T* delta_raw,
                  T* state_plus_delta_raw) const
  {
    const Eigen::Map<const ManifoldTemplate<T>> state(state_raw);
    const Eigen::Map<const TangentTemplate<T>>  delta(delta_raw);

    Eigen::Map<ManifoldTemplate<T>> state_plus_delta(state_plus_delta_raw);

    state_plus_delta = state + delta;

    std::cout << "state r " << state.coeffs()(0) << "\n";
    std::cout << "state i " << state.coeffs()(1) << "\n";
    std::cout << "state a " << state.angle() << "\n";

    std::cout << "delta " << delta.coeffs()(0) << "\n";

    std::cout << "state_plus_delta r " << state_plus_delta.coeffs()(0) << "\n";
    std::cout << "state_plus_delta i " << state_plus_delta.coeffs()(1) << "\n";
    std::cout << "state_plus_delta a " << state_plus_delta.angle() << "\n";

    return true;
  }

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
    return operator ()(state_raw, delta_raw, state_plus_delta_raw);

//    const Eigen::Map<const Manifold> state(state_raw);
//    const Eigen::Map<const Tangent>  delta(delta_raw);

//    Eigen::Map<Manifold> state_plus_delta(state_plus_delta_raw);

//    state_plus_delta = state + delta;

//    return true;
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

    state.rplus(tangent_zero_, tmp_out_, J_rplus_m_, J_rplus_t_);

    JacobianMap rplus_jacobian(rplus_jacobian_raw);
//    Eigen::Map<Jacobian> rplus_jacobian(rplus_jacobian_raw);
    rplus_jacobian = J_rplus_t_;

    rplus_jacobian_raw[0] = -1;
    rplus_jacobian_raw[1] = -1;

    return true;
  }

//  bool MultiplyByJacobian(const double *x, const int /*num_rows*/,
//                          const double *global_matrix,
//                          double *local_matrix) const override
//  {
////    ceres::Matrix jacobian(GlobalSize(), LocalSize());
//    Jacobian jacobian;
//    if (!ComputeJacobian(x, jacobian.data())) {
//      return false;
//    }

////    MatrixRef(local_matrix, num_rows, LocalSize()) =
////          ConstMatrixRef(global_matrix, num_rows, GlobalSize()) * jacobian;

//    const Eigen::Map<const Manifold> state(global_matrix);
//    Eigen::Map<Tangent> delta(local_matrix);

//    JacobianMap rplus_jacobian(x);

//    std::cout << "MultiplyByJacobian :\n"
//              << "state " << state
//              << "delta " << delta
//              << "rplus_jacobian " << rplus_jacobian
//              << "\n";

//    return true;
//  }

  virtual int GlobalSize() const override { return Manifold::RepSize; }
  virtual int LocalSize()  const override { return Manifold::DoF; }

protected:

  const Tangent tangent_zero_;

  mutable Manifold tmp_out_;
  mutable Jacobian J_rplus_m_, J_rplus_t_;
};

//using LocalParameterizationSO2 = LocalParameterization<SO2d>;
//using LocalParameterizationSO3 = LocalParameterization<SO3d>;
//using LocalParameterizationSE2 = LocalParameterization<SE2d>;
//using LocalParameterizationSE3 = LocalParameterization<SE3d>;

} /* namespace manif */

#endif /* _MANIF_MANIF_CERES_LOCAL_PARAMETRIZATION_H_ */
