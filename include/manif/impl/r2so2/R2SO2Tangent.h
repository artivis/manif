#ifndef _MANIF_MANIF_R2SO2TANGENT_H_
#define _MANIF_MANIF_R2SO2TANGENT_H_

#include "manif/impl/R2SO2/R2SO2Tangent_base.h"

#include <Eigen/Dense>

namespace manif
{
namespace internal
{

// Traits specialization

template <typename _Scalar>
struct traits<R2SO2Tangent<_Scalar>>
{
  using Scalar = _Scalar;

  using Manifold = R2SO2<_Scalar>;
  using Tangent  = R2SO2Tangent<_Scalar>;

  using Base = R2SO2TangentBase<_Scalar>;

  static constexpr int Dim     = ManifoldProperties<Base>::Dim;
  static constexpr int DoF     = ManifoldProperties<Base>::DoF;
  static constexpr int RepSize = DoF;

  using DataType = Eigen::Matrix<Scalar, DoF, 1>;
  using Jacobian = Eigen::Matrix<Scalar, DoF, DoF>;
  using LieAlg   = Eigen::Matrix<Scalar, 3, 3>;
};

} /* namespace internal */
} /* namespace manif */

namespace manif
{

///////////////
///         ///
/// Tangent ///
///         ///
///////////////

template <typename _Scalar>
struct R2SO2Tangent : R2SO2TangentBase<R2SO2Tangent<_Scalar>>
{
private:

  using Base = R2SO2TangentBase<R2SO2Tangent<_Scalar>>;
  using Type = R2SO2Tangent<_Scalar>;

public:

  MANIF_TANGENT_TYPEDEF

  R2SO2Tangent() = default;

  R2SO2Tangent(const DataType& v);
  R2SO2Tangent(const Scalar x, const Scalar y, const Scalar theta);

  /// Tangent common API

  DataType& coeffs();
  const DataType& coeffs() const;

  MANIF_INHERIT_TANGENT_API
  MANIF_INHERIT_TANGENT_OPERATOR

  /// R2SO2Tangent specific API

  using Base::angle;

protected:

  DataType data_;
};

MANIF_EXTRA_TANGENT_TYPEDEF(R2SO2Tangent);

template <typename _Scalar>
R2SO2Tangent<_Scalar>::R2SO2Tangent(const DataType& theta)
  : data_(theta)
{
  //
}

template <typename _Scalar>
R2SO2Tangent<_Scalar>::R2SO2Tangent(const Scalar x,
                                    const Scalar y,
                                    const Scalar theta)
  : R2SO2Tangent(DataType(x, y, theta))
{
  //
}

template <typename _Scalar>
typename R2SO2Tangent<_Scalar>::DataType&
R2SO2Tangent<_Scalar>::coeffs()
{
  return data_;
}

template <typename _Scalar>
const typename R2SO2Tangent<_Scalar>::DataType&
R2SO2Tangent<_Scalar>::coeffs() const
{
  return data_;
}

} /* namespace manif */

#endif /* _MANIF_MANIF_R2SO2TANGENT_H_ */
