#ifndef _MANIF_MANIF_JACOBIAN_H_
#define _MANIF_MANIF_JACOBIAN_H_

#include "lt/optional.hpp"

namespace manif {

// Foward declaration
template <typename _Derived> struct LieGroupBase;
template <typename _Derived> struct TangentBase;

namespace internal {

template <typename Out, typename In>
struct JacobianTraits
{
  using type = Eigen::Matrix<typename traits<Out>::Scalar,
                             traits<Out>::DoF,
                             traits<In>::DoF>;
};

template <typename Out, typename In>
struct JacobianTraits<Eigen::MatrixBase<Out>, Eigen::MatrixBase<In>>
{
  using type = Eigen::Matrix<typename Eigen::MatrixBase<Out>::Scalar,
                             Eigen::MatrixBase<Out>::SizeAtCompileTime,
                             Eigen::MatrixBase<In>::SizeAtCompileTime>;
};

template <typename Out, typename In>
struct JacobianTraits<Eigen::MatrixBase<Out>, In>
{
  using type = Eigen::Matrix<typename Eigen::MatrixBase<Out>::Scalar,
                             Eigen::MatrixBase<Out>::SizeAtCompileTime,
                             traits<In>::DoF>;
};

template <typename Out, typename In>
struct JacobianTraits<Out, Eigen::MatrixBase<In>>
{
  using type = Eigen::Matrix<typename Out::Scalar,
                             traits<Out>::DoF,
                             Eigen::MatrixBase<In>::SizeAtCompileTime>;
};

template <typename S1, int R1, int C1, typename S2, int R2, int C2>
struct JacobianTraits<Eigen::Matrix<S1, R1, C1>, Eigen::Matrix<S2, R2, C2>>
{
  using type = Eigen::Matrix<typename Eigen::Matrix<S1, R1, C1>::Scalar,
                             Eigen::Matrix<S1, R1, C1>::SizeAtCompileTime,
                             Eigen::Matrix<S2, R2, C2>::SizeAtCompileTime>;
};

template <typename Out, typename S2, int R2, int C2>
struct JacobianTraits<Out, Eigen::Matrix<S2, R2, C2>>
{
  using type = Eigen::Matrix<typename traits<Out>::Scalar,
                             traits<Out>::DoF,
                             Eigen::Matrix<S2, R2, C2>::SizeAtCompileTime>;
};

template <typename S1, int R1, int C1, typename In>
struct JacobianTraits<Eigen::Matrix<S1, R1, C1>, In>
{
  using type = Eigen::Matrix<typename Eigen::Matrix<S1, R1, C1>::Scalar,
                             Eigen::Matrix<S1, R1, C1>::SizeAtCompileTime,
                             traits<In>::DoF>;
};

} /* namespace internal */

// Helper
template <typename Out, typename In>
using Jacobian = typename internal::JacobianTraits<Out, In>::type;

template <typename Out, typename In>
using JacobianRef = Eigen::Ref<Jacobian<Out,In>>;

template <typename Out, typename In>
using OptJacobianRef = tl::optional<JacobianRef<Out,In>>;

} /* namespace manif */

#endif /* _MANIF_MANIF_EIGEN_H_ */
