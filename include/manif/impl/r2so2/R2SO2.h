#ifndef _MANIF_MANIF_R2SO2_H_
#define _MANIF_MANIF_R2SO2_H_

#include "manif/impl/R2SO2/R2SO2_base.h"

#include <Eigen/Dense>

namespace manif
{

// Forward declare for type traits specialization

template <typename _Scalar> struct R2SO2;
template <typename _Scalar> struct R2SO2Tangent;

namespace internal
{

// Traits specialization

template <typename _Scalar>
struct traits<R2SO2<_Scalar>>
{
  using Scalar = _Scalar;

  using Manifold = R2SO2<_Scalar>;
  using Tangent  = R2SO2Tangent<_Scalar>;

  using Base = R2SO2Base<R2SO2<_Scalar>>;

  static constexpr int Dim     = ManifoldProperties<Base>::Dim;
  static constexpr int DoF     = ManifoldProperties<Base>::DoF;
  static constexpr int N       = ManifoldProperties<Base>::N;
  static constexpr int RepSize = 4;

  using DataType = Eigen::Matrix<Scalar, RepSize, 1>;

  using Jacobian       = Eigen::Matrix<Scalar, DoF, DoF>;
  using Transformation = Eigen::Matrix<Scalar, N, N>;
  using Rotation       = Eigen::Matrix<Scalar, Dim, Dim>;
  using Translation    = Eigen::Matrix<Scalar, Dim, 1>;
  using Vector         = Eigen::Matrix<Scalar, DoF, 1>;
};

} /* namespace internal */
} /* namespace manif */

namespace manif
{

////////////////
///          ///
/// Manifold ///
///          ///
////////////////

template <typename _Scalar>
struct R2SO2 : R2SO2Base<R2SO2<_Scalar>>
{
private:

  using Base = R2SO2Base<R2SO2<_Scalar>>;
  using Type = R2SO2<_Scalar>;

public:

  MANIF_COMPLETE_MANIFOLD_TYPEDEF

  R2SO2()  = default;
  ~R2SO2() = default;

  R2SO2(const DataType& d);
  R2SO2(const Scalar x, const Scalar y, const Scalar theta);
  R2SO2(const Scalar x, const Scalar y, const Scalar real, const Scalar imag);

  /// Manifold common API

  const DataType& coeffs() const;

  MANIF_INHERIT_MANIFOLD_API

  /// R2SO2 specific API

  using Base::angle;
  using Base::real;
  using Base::imag;
  using Base::x;
  using Base::y;

protected:

  friend class ManifoldBase<R2SO2<Scalar>>;
  DataType& coeffs_nonconst();

  DataType data_;
};

MANIF_EXTRA_MANIFOLD_TYPEDEF(R2SO2)

template <typename _Scalar>
R2SO2<_Scalar>::R2SO2(const DataType& d)
  : data_(d)
{
  //
}

template <typename _Scalar>
R2SO2<_Scalar>::R2SO2(const Scalar x, const Scalar y, const Scalar theta)
  : R2SO2(DataType(x, y, cos(theta), sin(theta)))
{
  using std::cos;
  using std::sin;
}

template <typename _Scalar>
R2SO2<_Scalar>::R2SO2(const Scalar x, const Scalar y,
                  const Scalar real, const Scalar imag)
  : R2SO2(DataType(x, y, real, imag))
{
  //
}

template <typename _Scalar>
typename R2SO2<_Scalar>::DataType&
R2SO2<_Scalar>::coeffs_nonconst()
{
  return data_;
}

template <typename _Scalar>
const typename R2SO2<_Scalar>::DataType&
R2SO2<_Scalar>::coeffs() const
{
  return data_;
}

} /* namespace manif */

#endif /* _MANIF_MANIF_R2SO2_H_ */
