#ifndef _MANIF_MANIF_SO2_H_
#define _MANIF_MANIF_SO2_H_

#include "manif/impl/SO2_base.h"

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
  static constexpr int RepSize = 2;

  using ManifoldDataType = Eigen::Matrix<Scalar, RepSize, 1>;

  using JacobianMtoM = Eigen::Matrix<Scalar, RepSize, RepSize>;
  using JacobianMtoT = Eigen::Matrix<Scalar, DoF, RepSize>;

  using Transformation = Eigen::Matrix<Scalar, Dim+1, Dim+1>;

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
  using Base = SO2Base<SO2<_Scalar>>;

  using Tangent = typename Base::Tangent;

  MANIF_COMPLETE_MANIFOLD_TYPEDEF

  SO2()  = default;
  ~SO2() = default;

  SO2(const ManifoldDataType& d);
  SO2(const Scalar real, const Scalar imag);
  SO2(const Scalar theta);

  /// Manifold common API

protected:

  friend class ManifoldBase<SO2<Scalar>>;
  ManifoldDataType* data();

public:

  const ManifoldDataType* data() const;

  using Base::matrix;
  using Base::rotation;
  using Base::identity;
  using Base::random;
  using Base::inverse;
  using Base::rplus;
  using Base::lplus;
  using Base::rminus;
  using Base::lminus;
  using Base::lift;
  using Base::compose;

  /// SO2 specific API

  using Base::angle;

protected:

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
