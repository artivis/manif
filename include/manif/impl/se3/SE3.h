#ifndef _MANIF_MANIF_SE3_H_
#define _MANIF_MANIF_SE3_H_

#include "manif/impl/se3/SE3_base.h"

#include <Eigen/Core>

namespace manif
{

// Forward declare for type traits specialization

template <typename _Scalar> struct SE3;
template <typename _Scalar> struct SE3Tangent;

namespace internal
{

// Traits specialization

template <>
template <typename _Scalar>
struct traits<SE3<_Scalar>>
{
  using Scalar = _Scalar;

  template <typename T>
  using ManifoldTemplate = SE3<T>;

  using Manifold = SE3<_Scalar>;
  using Tangent  = SE3Tangent<_Scalar>;

  using Base = SE3Base<SE3<_Scalar>>;

  static constexpr int Dim = ManifoldProperties<Base>::Dim;
  static constexpr int DoF = ManifoldProperties<Base>::DoF;
  static constexpr int N   = ManifoldProperties<Base>::N;

  /// @todo would be nice to concat vec3 + quaternion
  using DataType = Eigen::Matrix<Scalar, 7, 1>;

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
struct SE3 : SE3Base<SE3<_Scalar>>
{
private:

  using Base = SE3Base<SE3<_Scalar>>;
  using Type = SE3<_Scalar>;

public:

  MANIF_COMPLETE_MANIFOLD_TYPEDEF
  using Translation = typename Base::Translation;

  SE3()  = default;
  ~SE3() = default;

  SE3(const DataType& d);

  SE3(const Translation& t,
      const Eigen::Quaternion<Scalar>& q);

  /// Manifold common API

  const DataType& coeffs() const;

  MANIF_INHERIT_MANIFOLD_API

  /// SE3 specific API

protected:

  friend class ManifoldBase<SE3<Scalar>>;
  DataType& coeffs_nonconst();

  DataType data_;
};

MANIF_EXTRA_MANIFOLD_TYPEDEF(SE3)

template <typename _Scalar>
SE3<_Scalar>::SE3(const DataType& d)
  : data_(d)
{
  //
}

template <typename _Scalar>
SE3<_Scalar>::SE3(const Translation& t,
                  const Eigen::Quaternion<Scalar>& q)
{
  coeffs_nonconst() << t, q.coeffs();
}

template <typename _Scalar>
typename SE3<_Scalar>::DataType&
SE3<_Scalar>::coeffs_nonconst()
{
  return data_;
}

template <typename _Scalar>
const typename SE3<_Scalar>::DataType&
SE3<_Scalar>::coeffs() const
{
  return data_;
}

} /* namespace manif */

#endif /* _MANIF_MANIF_SE3_H_ */
