#ifndef _MANIF_MANIF_R3SO3_H_
#define _MANIF_MANIF_R3SO3_H_

#include "manif/impl/R3SO3/R3SO3_base.h"

#include <Eigen/Core>

namespace manif
{

// Forward declare for type traits specialization

template <typename _Scalar> struct R3SO3;
template <typename _Scalar> struct R3SO3Tangent;

namespace internal
{

// Traits specialization

template <typename _Scalar>
struct traits<R3SO3<_Scalar>>
{
  using Scalar = _Scalar;

  using Manifold = R3SO3<_Scalar>;
  using Tangent  = R3SO3Tangent<_Scalar>;

  using Base = R3SO3Base<R3SO3<_Scalar>>;

  static constexpr int Dim = ManifoldProperties<Base>::Dim;
  static constexpr int DoF = ManifoldProperties<Base>::DoF;
  static constexpr int N   = ManifoldProperties<Base>::N;
  static constexpr int RepSize = 7;

  /// @todo would be nice to concat vec3 + quaternion
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
struct R3SO3 : R3SO3Base<R3SO3<_Scalar>>
{
private:

  using Base = R3SO3Base<R3SO3<_Scalar>>;
  using Type = R3SO3<_Scalar>;

public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  MANIF_COMPLETE_MANIFOLD_TYPEDEF
  using Translation = typename Base::Translation;
  using Quaternion = Eigen::Quaternion<Scalar>;

  R3SO3()  = default;
  ~R3SO3() = default;

  R3SO3(const DataType& d);

  R3SO3(const Translation& t,
      const Eigen::Quaternion<Scalar>& q);

  R3SO3(const Translation& t,
      const SO3<Scalar>& so3);

  /// Manifold common API

  const DataType& coeffs() const;

  MANIF_INHERIT_MANIFOLD_API

  /// R3SO3 specific API

protected:

  friend class ManifoldBase<R3SO3<Scalar>>;
  DataType& coeffs_nonconst();

  DataType data_;
};

MANIF_EXTRA_MANIFOLD_TYPEDEF(R3SO3)

template <typename _Scalar>
R3SO3<_Scalar>::R3SO3(const DataType& d)
  : data_(d)
{
  //
}

template <typename _Scalar>
R3SO3<_Scalar>::R3SO3(const Translation& t,
                  const Eigen::Quaternion<Scalar>& q)
  : R3SO3((DataType() << t, q.coeffs() ).finished())
{
  //
}

template <typename _Scalar>
R3SO3<_Scalar>::R3SO3(const Translation& t,
                  const SO3<Scalar>& so3)
  : R3SO3((DataType() << t, so3.coeffs() ).finished())
{
  //
}

template <typename _Scalar>
typename R3SO3<_Scalar>::DataType&
R3SO3<_Scalar>::coeffs_nonconst()
{
  return data_;
}

template <typename _Scalar>
const typename R3SO3<_Scalar>::DataType&
R3SO3<_Scalar>::coeffs() const
{
  return data_;
}

} /* namespace manif */

#endif /* _MANIF_MANIF_R3SO3_H_ */
