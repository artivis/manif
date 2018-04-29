#ifndef _MANIF_MANIF_SO3_H_
#define _MANIF_MANIF_SO3_H_

#include "manif/impl/so3/SO3_base.h"

namespace manif
{

// Forward declare for type traits specialization

template <typename _Scalar> struct SO3;
template <typename _Scalar> struct SO3Tangent;

namespace internal
{

// Traits specialization

template <>
template <typename _Scalar>
struct traits<SO3<_Scalar>>
{
  using Scalar = _Scalar;

  using Manifold = SO3<_Scalar>;
  using Tangent  = SO3Tangent<_Scalar>;

  using Base = SO3Base<SO3<_Scalar>>;

  static constexpr int Dim     = ManifoldProperties<Base>::Dim;
  static constexpr int DoF     = ManifoldProperties<Base>::DoF;
  static constexpr int N       = ManifoldProperties<Base>::N;
  static constexpr int RepSize = 4;

  using DataType = Eigen::Quaternion<Scalar>;
//  using DataType = Eigen::Matrix<Scalar, 4, 1>;

  using Jacobian = Eigen::Matrix<Scalar, DoF, DoF>;

  using Transformation = Eigen::Matrix<Scalar, N, N>;

  using Rotation = Eigen::Matrix<Scalar, Dim, Dim>;

  using Vector = Eigen::Matrix<Scalar, Dim, 1>;
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
struct SO3 : SO3Base<SO3<_Scalar>>
{
private:

  using Base = SO3Base<SO3<_Scalar>>;
  using Type = SO3<_Scalar>;

public:

  MANIF_COMPLETE_MANIFOLD_TYPEDEF

  SO3()  = default;
  ~SO3() = default;

  SO3(const DataType& d);

  SO3(const Scalar x, const Scalar y,
      const Scalar z, const Scalar w);

  SO3(const Eigen::AngleAxis<Scalar>& angle_axis);

  const DataType* data() const;

  MANIF_INHERIT_MANIFOLD_API

protected:

  friend class ManifoldBase<SO3<Scalar>>;
  DataType* data();

  DataType data_;
};

MANIF_EXTRA_MANIFOLD_TYPEDEF(SO3)

/// SO3 functions definitions

template <typename _Scalar>
SO3<_Scalar>::SO3(const DataType& d)
  : data_(d)
{
  //
}

template <typename _Scalar>
SO3<_Scalar>::SO3(const Scalar x, const Scalar y,
                  const Scalar z, const Scalar w)
  : data_(w, x,y, z)
{
  //
}

template <typename _Scalar>
SO3<_Scalar>::SO3(const Eigen::AngleAxis<Scalar>& angle_axis)
  : data_(angle_axis)
{

}

template <typename _Scalar>
typename SO3<_Scalar>::DataType*
SO3<_Scalar>::data()
{
  return &data_;
}

template <typename _Scalar>
const typename SO3<_Scalar>::DataType*
SO3<_Scalar>::data() const
{
  return &data_;
}

} /* namespace manif */

#endif /* _MANIF_MANIF_SO3_H_ */
