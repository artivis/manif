#ifndef _MANIF_MANIF_SE3_H_
#define _MANIF_MANIF_SE3_H_

#include "manif/impl/se3/SE3_base.h"

#include <Eigen/Core>

namespace manif {

// Forward declare for type traits specialization

template <typename _Scalar> struct SE3;
template <typename _Scalar> struct SE3Tangent;

namespace internal {

// Traits specialization

template <typename _Scalar>
struct traits<SE3<_Scalar>>
{
  using Scalar = _Scalar;

  using LieGroup = SE3<_Scalar>;
  using Tangent  = SE3Tangent<_Scalar>;

  using Base = SE3Base<SE3<_Scalar>>;

  static constexpr int Dim = LieGroupProperties<Base>::Dim;
  static constexpr int DoF = LieGroupProperties<Base>::DoF;
  static constexpr int N   = LieGroupProperties<Base>::N;
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
/// LieGroup ///
///          ///
////////////////

template <typename _Scalar>
struct SE3 : SE3Base<SE3<_Scalar>>
{
private:

  using Base = SE3Base<SE3<_Scalar>>;
  using Type = SE3<_Scalar>;

public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  MANIF_COMPLETE_GROUP_TYPEDEF
  using Translation = typename Base::Translation;
  using Quaternion = Eigen::Quaternion<Scalar>;

  SE3()  = default;
  ~SE3() = default;

  SE3(const DataType& d);

  SE3(const Translation& t,
      const Eigen::Quaternion<Scalar>& q);

  SE3(const Translation& t,
      const SO3<Scalar>& so3);

  /// LieGroup common API

  const DataType& coeffs() const;

  MANIF_INHERIT_GROUP_API

  /// SE3 specific API

protected:

  friend class LieGroupBase<SE3<Scalar>>;
  DataType& coeffs_nonconst();

  DataType data_;
};

MANIF_EXTRA_GROUP_TYPEDEF(SE3)

template <typename _Scalar>
SE3<_Scalar>::SE3(const DataType& d)
  : data_(d)
{
  //
}

template <typename _Scalar>
SE3<_Scalar>::SE3(const Translation& t,
                  const Eigen::Quaternion<Scalar>& q)
  : SE3((DataType() << t, q.coeffs() ).finished())
{
  //
}

template <typename _Scalar>
SE3<_Scalar>::SE3(const Translation& t,
                  const SO3<Scalar>& so3)
  : SE3((DataType() << t, so3.coeffs() ).finished())
{
  //
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
