#ifndef _MANIF_MANIF_SO2_H_
#define _MANIF_MANIF_SO2_H_

#include "manif/impl/so2/SO2_base.h"

#include <Eigen/Core>

namespace manif
{

// Forward declare for type traits specialization

template <typename _Scalar> struct SO2;
template <typename _Scalar> struct SO2Tangent;

namespace internal
{

// Traits specialization

template <>
template <typename _Scalar>
struct traits<SO2<_Scalar>>
{
  using Scalar = _Scalar;

  using Manifold = SO2<_Scalar>;
  using Tangent  = SO2Tangent<_Scalar>;

  using Base = SO2Base<SO2<_Scalar>>;

  static constexpr int Dim     = ManifoldProperties<Base>::Dim;
  static constexpr int DoF     = ManifoldProperties<Base>::DoF;
  static constexpr int N       = ManifoldProperties<Base>::N;
  static constexpr int RepSize = 2;

  using ManifoldDataType = Eigen::Matrix<Scalar, RepSize, 1>;

  using Jacobian = Eigen::Matrix<Scalar, DoF, DoF>;

  using Transformation = Eigen::Matrix<Scalar, N, N>;

  using Rotation = Eigen::Matrix<Scalar, Dim, Dim>;
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
struct SO2 : SO2Base<SO2<_Scalar>>
{
private:

  using Base = SO2Base<SO2<_Scalar>>;
  using Type = SO2<_Scalar>;

public:

  MANIF_COMPLETE_MANIFOLD_TYPEDEF

  SO2()  = default;
  ~SO2() = default;

  SO2(const ManifoldDataType& d);
  SO2(const Scalar real, const Scalar imag);
  SO2(const Scalar theta);

  /// Manifold common API

  const ManifoldDataType* data() const;

  MANIF_INHERIT_MANIFOLD_API

  /// SO2 specific API

  using Base::angle;

protected:

  friend class ManifoldBase<SO2<Scalar>>;
  ManifoldDataType* data();

  ManifoldDataType data_;
};

MANIF_EXTRA_MANIFOLD_TYPEDEF(SO2)

template <typename _Scalar>
SO2<_Scalar>::SO2(const ManifoldDataType& d)
  : data_(d)
{
  //
}

template <typename _Scalar>
SO2<_Scalar>::SO2(const Scalar real, const Scalar imag)
  : SO2(ManifoldDataType(real, imag))
{
  //
}

template <typename _Scalar>
SO2<_Scalar>::SO2(const Scalar theta)
  : SO2(cos(theta), sin(theta))
{
  using std::cos;
  using std::sin;
}

template <typename _Scalar>
typename SO2<_Scalar>::ManifoldDataType*
SO2<_Scalar>::data()
{
  return &data_;
}

template <typename _Scalar>
const typename SO2<_Scalar>::ManifoldDataType*
SO2<_Scalar>::data() const
{
  return &data_;
}

} /* namespace manif */

#endif /* _MANIF_MANIF_SO2_H_ */
